"""Fails when a standard type math's model uses is not in Rant's type list, types/stdtypes.json."""
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "bindgen"))
import model as model_module  # noqa: E402
from python_backend import own_names  # noqa: E402


def main(types_path, clang, listing):
    m = model_module.read_model(ROOT, types_path, clang)
    known = {t["name"] for t in json.load(open(listing, encoding="utf-8"))["types"]}
    own = own_names(m)
    used = {model_module.short(s.name) for s in m.structs.values() if s.std}
    used |= {model_module.short(e.name) for e in m.enums.values() if e.std}
    used |= {me.type for me in m.methods if me.type not in own}
    for o in m.operators:
        used |= {o.left, o.right} - {"", "double"}
    missing = sorted(used - known)
    if missing:
        print("not in Rant's type list: " + ", ".join(missing))
        return 1
    print(f"{len(used)} standard types, all in Rant's list")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:4]))
