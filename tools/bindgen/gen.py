#!/usr/bin/env python3
"""Writes every language's binding of Rant Math from its C headers.

    gen.py --types <Rant's src/common/stdtypes.h>           writes each generated file
    gen.py --types <...> --check                            fails when a checked in file is stale
    gen.py --types <...> --record-source <out.c>            writes the C program that records the vectors
    gen.py --types <...> --record-answers <answers> --write-vectors
                                                            writes tests/bindings/vectors.json from its answers
    gen.py --types <...> --record-answers <answers> --check-vectors
                                                            fails when an answer differs from the checked in file
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import model as model_module
import vectors

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
VECTORS = os.path.join(ROOT, "tests", "bindings", "vectors.json")


def outputs(model):
    """Each generated file as (path under the repo, text). Backends join as they land."""
    files = []
    for name in ("cpp_backend", "python_backend", "csharp_backend", "rust_backend", "web_backend", "quickjs_backend"):
        try:
            backend = __import__(name)
        except ModuleNotFoundError:
            continue
        files += backend.generate(model)
    return files


def read_answers(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def check_vectors(cases, answers, path):
    with open(path, encoding="utf-8") as f:
        recorded = json.load(f)
    stored = []
    for entry in recorded["functions"]:
        stored += [(entry["f"], entry["tol"], c) for c in entry["cases"]]
    if len(stored) != len(answers):
        print(f"{len(answers)} answers now, {len(stored)} in {path}: run gen.py --write-vectors")
        return 1
    bad = 0
    for (f, tol, case), answer in zip(stored, answers):
        want = {k: v for k, v in case.items() if k != "a"}
        if not vectors.close(want, answer, tol):
            print(f"{f}: {json.dumps(case['a'])[:200]}\n  recorded {json.dumps(want)[:400]}\n  now      {json.dumps(answer)[:400]}")
            bad += 1
    print(f"{len(stored) - bad} of {len(stored)} cases agree with the recorded answers")
    return 1 if bad else 0


def coverage(cases, answers):
    """Which paths the cases reach, so a function that always succeeds is seen and asked to fail."""
    seen = {}
    for (f, _), a in zip(cases, answers):
        s = seen.setdefault(f.name, [f, set()])
        if "e" in a:
            s[1].add("error")
        elif a.get("v") is None and f.outputs and f.errors == "optional":
            s[1].add("none")
        else:
            s[1].add("value")
    lines = []
    for name, (f, got) in seen.items():
        if f.errors == "throw" and "error" not in got:
            lines.append(f"{name}: never fails")
        if f.errors == "throw" and "value" not in got:
            lines.append(f"{name}: never succeeds")
        if f.errors == "optional" and got != {"none", "value"}:
            lines.append(f"{name}: only {sorted(got)}")
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--types", required=True, help="Rant's src/common/stdtypes.h")
    ap.add_argument("--check", action="store_true", help="fail when a checked in file is stale")
    ap.add_argument("--record-source", help="write the C program that records the vectors here")
    ap.add_argument("--record-answers", help="the file that program wrote")
    ap.add_argument("--write-vectors", action="store_true")
    ap.add_argument("--check-vectors", action="store_true")
    ap.add_argument("--coverage", action="store_true")
    a = ap.parse_args()
    model = model_module.read_model(ROOT, a.types)

    if a.record_source:
        text = vectors.write_record_program(model, vectors.build_cases(model))
        with open(a.record_source, "w", newline="\n", encoding="utf-8") as f:
            f.write(text)
        return 0
    if a.record_answers:
        cases = vectors.build_cases(model)
        answers = read_answers(a.record_answers)
        if a.coverage:
            print("\n".join(coverage(cases, answers)))
        if a.write_vectors:
            os.makedirs(os.path.dirname(VECTORS), exist_ok=True)
            with open(VECTORS, "w", newline="\n", encoding="utf-8") as f:
                f.write(vectors.vectors_file(model, cases, answers))
            print(f"wrote {VECTORS}, {len(answers)} cases")
        if a.check_vectors:
            return check_vectors(cases, answers, VECTORS)
        return 0

    stale = 0
    for rel, text in outputs(model):
        path = os.path.join(ROOT, rel)
        if a.check:
            try:
                with open(path, encoding="utf-8", newline="") as f:
                    now = f.read().replace("\r\n", "\n")
            except OSError:
                now = None
            if now != text:
                print(f"stale: {rel}")
                stale += 1
        else:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", newline="\n", encoding="utf-8") as f:
                f.write(text)
            print(f"wrote {rel}")
    if a.check and stale:
        print("run tools/bindgen/gen.py to write them")
    return 1 if stale else 0


if __name__ == "__main__":
    sys.exit(main())
