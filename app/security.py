"""
Security Module for BlueOS LiDAR SLAM Extension

Addresses critical blind spots:
- Path traversal prevention
- API authentication
- Input validation
- Rate limiting
"""

import re
import os
import hmac
import hashlib
import secrets
import time
import logging
from functools import wraps
from typing import Optional, Callable
from flask import request, jsonify, g

logger = logging.getLogger(__name__)

# Token storage (in production, use Redis or database)
API_TOKENS = {}
RATE_LIMIT_STORE = {}


def generate_api_token() -> str:
    """Generate a secure API token"""
    return secrets.token_urlsafe(32)


def hash_token(token: str) -> str:
    """Hash token for secure storage"""
    return hashlib.sha256(token.encode()).hexdigest()


def validate_path_component(name: str) -> bool:
    """
    Validate path component to prevent traversal attacks

    Allows: alphanumeric, underscore, hyphen, dot (not at start)
    Rejects: path separators, null bytes, control chars, ..
    """
    if not name or len(name) > 255:
        return False

    if name in ('.', '..'):
        return False

    if '\0' in name or '/' in name or '\\' in name:
        return False

    # Only allow safe characters
    pattern = r'^[a-zA-Z0-9][a-zA-Z0-9_\-\.]*$'
    return bool(re.match(pattern, name))


def sanitize_filename(name: str) -> Optional[str]:
    """
    Sanitize filename, returning None if invalid
    """
    if not validate_path_component(name):
        logger.warning(f"Invalid filename rejected: {repr(name)}")
        return None
    return name


def safe_join(base_dir: str, filename: str) -> Optional[str]:
    """
    Safely join base directory with filename, preventing traversal

    Returns None if the resulting path escapes base_dir
    """
    if not validate_path_component(filename):
        return None

    # Resolve to absolute paths
    base = os.path.abspath(base_dir)
    result = os.path.abspath(os.path.join(base, filename))

    # Verify result is under base
    if not result.startswith(base + os.sep) and result != base:
        logger.warning(f"Path traversal attempt blocked: {filename}")
        return None

    return result


class RateLimiter:
    """Simple in-memory rate limiter"""

    def __init__(self, requests_per_minute: int = 60):
        self.rpm = requests_per_minute
        self.window = 60  # seconds

    def is_allowed(self, client_id: str) -> bool:
        """Check if client is within rate limit"""
        now = time.time()

        if client_id not in RATE_LIMIT_STORE:
            RATE_LIMIT_STORE[client_id] = []

        # Clean old entries
        RATE_LIMIT_STORE[client_id] = [
            ts for ts in RATE_LIMIT_STORE[client_id]
            if now - ts < self.window
        ]

        if len(RATE_LIMIT_STORE[client_id]) >= self.rpm:
            return False

        RATE_LIMIT_STORE[client_id].append(now)
        return True


rate_limiter = RateLimiter(requests_per_minute=120)


def get_client_id() -> str:
    """Get client identifier for rate limiting"""
    forwarded = request.headers.get('X-Forwarded-For')
    if forwarded:
        return forwarded.split(',')[0].strip()
    return request.remote_addr or 'unknown'


def require_auth(f: Callable) -> Callable:
    """
    Decorator requiring API authentication

    Accepts:
    - Bearer token in Authorization header
    - X-API-Key header
    - api_key query parameter (for WebSocket compat)
    """
    @wraps(f)
    def decorated(*args, **kwargs):
        token = None

        # Check Authorization header
        auth_header = request.headers.get('Authorization', '')
        if auth_header.startswith('Bearer '):
            token = auth_header[7:]

        # Check X-API-Key header
        if not token:
            token = request.headers.get('X-API-Key')

        # Check query parameter (last resort)
        if not token:
            token = request.args.get('api_key')

        if not token:
            return jsonify({'error': 'Authentication required'}), 401

        # Validate token
        token_hash = hash_token(token)
        if token_hash not in API_TOKENS:
            logger.warning(f"Invalid token attempt from {get_client_id()}")
            return jsonify({'error': 'Invalid token'}), 401

        g.authenticated = True
        g.token_info = API_TOKENS[token_hash]
        return f(*args, **kwargs)

    return decorated


def require_rate_limit(f: Callable) -> Callable:
    """Decorator for rate limiting"""
    @wraps(f)
    def decorated(*args, **kwargs):
        client_id = get_client_id()
        if not rate_limiter.is_allowed(client_id):
            logger.warning(f"Rate limit exceeded for {client_id}")
            return jsonify({'error': 'Rate limit exceeded'}), 429
        return f(*args, **kwargs)

    return decorated


def validate_json_input(required_fields: list = None, optional_fields: list = None):
    """Decorator for validating JSON input"""
    def decorator(f: Callable) -> Callable:
        @wraps(f)
        def decorated(*args, **kwargs):
            data = request.get_json(silent=True)

            if required_fields and not data:
                return jsonify({'error': 'JSON body required'}), 400

            if data and required_fields:
                missing = [field for field in required_fields if field not in data]
                if missing:
                    return jsonify({
                        'error': f'Missing required fields: {", ".join(missing)}'
                    }), 400

            return f(*args, **kwargs)
        return decorated
    return decorator


def init_default_token() -> str:
    """
    Initialize a default API token from environment or generate new
    Returns the token (only on first call)
    """
    env_token = os.environ.get('LIDAR_API_TOKEN')

    if env_token:
        token_hash = hash_token(env_token)
        API_TOKENS[token_hash] = {
            'name': 'env_default',
            'created': time.time(),
            'permissions': ['all']
        }
        logger.info("API token loaded from environment")
        return '[set via LIDAR_API_TOKEN]'

    # Generate new token
    token = generate_api_token()
    token_hash = hash_token(token)
    API_TOKENS[token_hash] = {
        'name': 'auto_generated',
        'created': time.time(),
        'permissions': ['all']
    }

    logger.info(f"Generated API token: {token}")
    logger.info("Set LIDAR_API_TOKEN environment variable to use a persistent token")

    return token


# Public routes that don't require auth
PUBLIC_ROUTES = frozenset([
    '/',
    '/api/register_service',
    '/api/health',
])


def is_public_route(path: str) -> bool:
    """Check if route is public"""
    return path in PUBLIC_ROUTES
