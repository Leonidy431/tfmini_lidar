"""
Tests for the security module: path traversal protection, authentication,
rate limiting, and input validation.

These tests only depend on Flask (no numpy/open3d), so they run in a minimal
environment.
"""

import sys
import os

import pytest
from flask import Flask, jsonify

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import security
from app.security import (
    validate_path_component,
    sanitize_filename,
    safe_join,
    hash_token,
    generate_api_token,
    require_auth,
    require_rate_limit,
    RateLimiter,
    is_public_route,
)


class TestPathValidation:
    """Path traversal protection."""

    @pytest.mark.parametrize("name", [
        "map1", "my-map", "map_2024", "scan.ply", "a", "A1_b-2.npy",
    ])
    def test_valid_names(self, name):
        assert validate_path_component(name) is True

    @pytest.mark.parametrize("name", [
        "../etc/passwd",
        "..",
        ".",
        "foo/bar",
        "foo\\bar",
        "/etc/passwd",
        "map\0name",
        "",
        ".hidden",          # leading dot not allowed
        "-flag",            # must start alphanumeric
        "a" * 256,          # too long
    ])
    def test_invalid_names(self, name):
        assert validate_path_component(name) is False

    def test_sanitize_filename_valid(self):
        assert sanitize_filename("good_name") == "good_name"

    def test_sanitize_filename_invalid(self):
        assert sanitize_filename("../evil") is None


class TestSafeJoin:
    """safe_join must keep results inside the base directory."""

    def test_normal_join(self, tmp_path):
        result = safe_join(str(tmp_path), "map1")
        assert result is not None
        assert result.startswith(str(tmp_path))

    def test_traversal_blocked(self, tmp_path):
        assert safe_join(str(tmp_path), "../secret") is None

    def test_absolute_blocked(self, tmp_path):
        assert safe_join(str(tmp_path), "/etc/passwd") is None

    def test_nested_traversal_blocked(self, tmp_path):
        assert safe_join(str(tmp_path), "a/../../b") is None


class TestTokens:
    def test_generate_unique(self):
        t1 = generate_api_token()
        t2 = generate_api_token()
        assert t1 != t2
        assert len(t1) > 20

    def test_hash_deterministic(self):
        assert hash_token("abc") == hash_token("abc")
        assert hash_token("abc") != hash_token("abd")


class TestRateLimiter:
    def test_allows_within_limit(self):
        limiter = RateLimiter(requests_per_minute=3)
        cid = "client-a"
        assert limiter.is_allowed(cid) is True
        assert limiter.is_allowed(cid) is True
        assert limiter.is_allowed(cid) is True

    def test_blocks_over_limit(self):
        limiter = RateLimiter(requests_per_minute=2)
        cid = "client-b"
        assert limiter.is_allowed(cid) is True
        assert limiter.is_allowed(cid) is True
        assert limiter.is_allowed(cid) is False

    def test_independent_clients(self):
        limiter = RateLimiter(requests_per_minute=1)
        assert limiter.is_allowed("c1") is True
        assert limiter.is_allowed("c2") is True


class TestPublicRoutes:
    def test_public(self):
        assert is_public_route("/") is True
        assert is_public_route("/api/health") is True
        assert is_public_route("/api/register_service") is True

    def test_not_public(self):
        assert is_public_route("/api/start") is False
        assert is_public_route("/api/maps/x/delete") is False


@pytest.fixture
def auth_app():
    """A minimal Flask app with an authenticated route and a valid token."""
    app = Flask(__name__)

    # Reset global state between tests
    security.API_TOKENS.clear()
    security.RATE_LIMIT_STORE.clear()

    token = "test-token-123"
    security.API_TOKENS[hash_token(token)] = {
        'name': 'test', 'created': 0, 'permissions': ['all']
    }

    @app.route('/protected', methods=['POST'])
    @require_auth
    def protected():
        return jsonify({'ok': True})

    @app.route('/limited')
    @require_rate_limit
    def limited():
        return jsonify({'ok': True})

    app.config['TEST_TOKEN'] = token
    return app


class TestRequireAuth:
    def test_no_token_rejected(self, auth_app):
        client = auth_app.test_client()
        resp = client.post('/protected')
        assert resp.status_code == 401

    def test_bad_token_rejected(self, auth_app):
        client = auth_app.test_client()
        resp = client.post('/protected', headers={'Authorization': 'Bearer wrong'})
        assert resp.status_code == 401

    def test_valid_bearer_accepted(self, auth_app):
        client = auth_app.test_client()
        token = auth_app.config['TEST_TOKEN']
        resp = client.post('/protected', headers={'Authorization': f'Bearer {token}'})
        assert resp.status_code == 200
        assert resp.get_json()['ok'] is True

    def test_valid_apikey_header_accepted(self, auth_app):
        client = auth_app.test_client()
        token = auth_app.config['TEST_TOKEN']
        resp = client.post('/protected', headers={'X-API-Key': token})
        assert resp.status_code == 200

    def test_valid_query_param_accepted(self, auth_app):
        client = auth_app.test_client()
        token = auth_app.config['TEST_TOKEN']
        resp = client.post(f'/protected?api_key={token}')
        assert resp.status_code == 200


class TestRequireRateLimit:
    def test_rate_limit_enforced(self, auth_app):
        # Shrink the shared limiter for the test
        original = security.rate_limiter.rpm
        security.rate_limiter.rpm = 2
        try:
            client = auth_app.test_client()
            assert client.get('/limited').status_code == 200
            assert client.get('/limited').status_code == 200
            assert client.get('/limited').status_code == 429
        finally:
            security.rate_limiter.rpm = original


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
