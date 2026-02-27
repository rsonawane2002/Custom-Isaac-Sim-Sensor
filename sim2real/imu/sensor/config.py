import json
from pathlib import Path
import omni.kit.app


def get_ext_root(ext_id: str) -> Path:
    """Return the root folder of this extension, regardless of where it is installed."""
    mgr = omni.kit.app.get_app().get_extension_manager()
    return Path(mgr.get_extension_path(ext_id))


def load_model_config(ext_root: Path, model: str) -> dict:
    """
    Load a model's config from data/models/<model>.json.
    Raises FileNotFoundError if the JSON does not exist.
    """
    p = ext_root / "data" / "models" / f"{model}.json"
    if not p.exists():
        raise FileNotFoundError(
            f"[Sim2Real IMU] No config file found for model '{model}' at {p}\n"
            f"Create {p.name} in the data/models/ folder to add this model."
        )
    config = json.loads(p.read_text())
    # Remove non-numeric keys that are metadata only
    #config.pop("model_id", None)
    #config.pop("noise_profile", None)
    return config
