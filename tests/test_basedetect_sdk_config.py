from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def load_sdk_config_module():
    module_path = PROJECT_ROOT / "BaseDetect/sdk/config.py"
    spec = importlib.util.spec_from_file_location("basedetect_sdk_config", module_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_load_settings_resolves_repo_root_weights_and_config_relative_camera_yaml():
    config_module = load_sdk_config_module()

    settings = config_module.load_settings(
        PROJECT_ROOT / "BaseDetect/configs/basedetect_sdk.yaml"
    )

    status_profile = settings.profiles["status_competition"]
    base_coord_profile = settings.profiles["base_coord_competition"]
    assert status_profile.weights == str(PROJECT_ROOT / "BaseDetect/models/data-3tag-v2.pt")
    assert base_coord_profile.weights == str(PROJECT_ROOT / "BaseDetect/models/data-v8-base.pt")
    assert base_coord_profile.camera_yaml == PROJECT_ROOT / "BaseDetect/configs/camera.yaml"
