"""Run all tests without needing pytest:  python run_tests.py
(With pytest installed you can also use:  python -m pytest -q)"""
import importlib
import sys
import time
import traceback

MODULES = ["tests.test_planner", "tests.test_navigation"]


def main() -> int:
    sys.path.insert(0, ".")
    passed = failed = 0
    t_all = time.perf_counter()
    for mod in MODULES:
        m = importlib.import_module(mod)
        path = mod.replace(".", "/") + ".py"
        for name in sorted(n for n in dir(m) if n.startswith("test_")):
            t0 = time.perf_counter()
            try:
                getattr(m, name)()
                passed += 1
                print(f"{path}::{name} ... PASS ({time.perf_counter() - t0:.2f}s)")
            except Exception:
                failed += 1
                print(f"{path}::{name} ... FAIL")
                traceback.print_exc()
    print("-" * 70)
    print(f"{passed} passed, {failed} failed in {time.perf_counter() - t_all:.1f}s")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
