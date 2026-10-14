"""Load local bridge source directly, bypassing Kit's persistent import caches."""

import importlib.util
import sys
from pathlib import Path


def load_bridge_modules(project):
    project = Path(project).resolve()
    package_dir = project / "px4_isaac"
    names = ("px4_isaac", "px4_isaac.model", "px4_isaac.transport", "px4_isaac.bridge")
    previous = {name: sys.modules[name] for name in names if name in sys.modules}
    for name in names:
        sys.modules.pop(name, None)
    try:
        for name in names:
            path = package_dir / (
                "__init__.py" if name == "px4_isaac" else name.rsplit(".", 1)[1] + ".py"
            )
            spec = importlib.util.spec_from_file_location(name, str(path))
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            # Do not call loader.exec_module/get_code: Kit can return old cached
            # code despite invalidate_caches/reload. Compile the actual file bytes.
            exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
            if "." in name:
                setattr(sys.modules["px4_isaac"], name.rsplit(".", 1)[1], module)
        return sys.modules["px4_isaac.bridge"]
    except Exception:
        for name in names:
            sys.modules.pop(name, None)
        sys.modules.update(previous)
        raise
