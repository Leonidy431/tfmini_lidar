"""
Tests for the heading-source honesty flag added to health reporting
(Physics Audit C3): the app must surface "no heading source" instead of
silently mapping every reading onto a single world axis.
"""

import sys
import os

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.main import LiDARSLAMApplication


@pytest.fixture
def app_instance():
    return LiDARSLAMApplication()


class TestHeadingHealthFlag:
    def test_idle_mode_does_not_require_heading(self, app_instance):
        health = app_instance.get_health()
        assert health['state'] == 'healthy'
        assert 'no_heading_source' not in health['reasons']
        assert health['heading_source_active'] is False

    def test_mapping_mode_flags_missing_heading(self, app_instance):
        app_instance.mode = LiDARSLAMApplication.MODE_MAPPING
        health = app_instance.get_health()
        assert health['state'] == 'degraded'
        assert 'no_heading_source' in health['reasons']

    def test_scanning_mode_flags_missing_heading(self, app_instance):
        app_instance.mode = LiDARSLAMApplication.MODE_SCANNING
        health = app_instance.get_health()
        assert 'no_heading_source' in health['reasons']

    def test_set_heading_clears_the_flag(self, app_instance):
        app_instance.mode = LiDARSLAMApplication.MODE_MAPPING
        assert 'no_heading_source' in app_instance.get_health()['reasons']

        app_instance.set_heading(90.0)

        health = app_instance.get_health()
        assert 'no_heading_source' not in health['reasons']
        assert health['heading_source_active'] is True
        assert app_instance.current_heading == 90.0

    def test_set_heading_wraps_to_0_360(self, app_instance):
        app_instance.set_heading(370.0)
        assert app_instance.current_heading == 10.0
        app_instance.set_heading(-10.0)
        assert app_instance.current_heading == 350.0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
