#!/usr/bin/env python3
"""Generates src/ui/MjcfDocs.cpp (attribute tooltips) from MuJoCo's documentation.

Usage: python3 tools/gen_mjcf_docs.py <mujoco>/doc/modeling.rst <mujoco>/doc/XMLreference.rst > src/ui/MjcfDocs.cpp

modeling.rst comes first: it holds the attributes common to all sensors, which the XML reference
only cross-references ("See CSensor").

Keys look like "geom.friction", "joint.armature", "option.timestep", "position.kp".
"""
import re
import sys

# anchor prefix -> key prefix
PREFIXES = {
    "body-inertial-": "inertial.",
    "body-joint-": "joint.",
    "body-freejoint-": "freejoint.",
    "body-geom-": "geom.",
    "body-site-": "site.",
    "body-camera-": "camera.",
    "body-light-": "light.",
    "body-": "body.",
    "option-flag-": "flag.",
    "option-": "option.",
    "compiler-": "compiler.",
    "asset-texture-": "texture.",
    "asset-material-": "material.",
    "asset-hfield-": "hfield.",
    "contact-exclude-": "exclude.",
    "contact-pair-": "pair.",
    "equality-connect-": "connect.",
    "equality-weld-": "weld.",
    "equality-joint-": "eqjoint.",
    "equality-tendon-": "eqtendon.",
    "tendon-spatial-": "spatial.",
    "tendon-fixed-": "fixed.",
    "spatial-geom-": "spatialgeom.",
    "spatial-pulley-": "pulley.",
    "fixed-joint-": "fixedjoint.",
    "visual-headlight-": "headlight.",
    "visual-global-": "global.",
    "visual-quality-": "quality.",
    "actuator-general-": "general.",
    "actuator-motor-": "motor.",
    "actuator-position-": "position.",
    "actuator-velocity-": "velocity.",
    "actuator-intvelocity-": "intvelocity.",
    "actuator-damper-": "damper.",
    "actuator-cylinder-": "cylinder.",
    "actuator-muscle-": "muscle.",
    "actuator-adhesion-": "adhesion.",
    "sensor-jointpos-": "sensor.",
    "sensor-contact-": "contactsensor.",
    "sensor-insidesite-": "insidesite.",
    "sensor-": "sensor.",
    "asset-mesh-": "mesh.",
    "size-": "size.",
}
SKIP_SUBSTR = ("body-attach", "body-frame", "body-composite", "body-flexcomp", "body-replicate", "body-plugin")


def clean(text: str) -> str:
    t = text
    t = re.sub(r":ref:`([^`<]*?)\s*<[^>]*>`", r"\1", t)
    t = re.sub(r":ref:`([^`]*)`", r"\1", t)
    t = re.sub(r":(?:at|el|at-val|math|func|class|attr)[a-z-]*:`([^`]*)`", r"\1", t)
    t = re.sub(r"`([^`<]*?)\s*<[^>]*>`__?", r"\1", t)
    t = re.sub(r"``([^`]*)``", r"\1", t)
    t = re.sub(r"\|[^|\s]+\|", "", t)
    t = t.replace("\\", "")
    t = re.sub(r"\s+", " ", t).strip()
    return t


def main(paths) -> None:
    docs = {}
    for path in paths:
        parse(path, docs)
    write(docs)


def parse(path: str, docs: dict) -> None:
    lines = open(path, encoding="utf-8").read().splitlines()
    i = 0
    n = len(lines)
    while i < n:
        m = re.match(r"^\.\. _([a-z0-9_\-]+):\s*$", lines[i])
        if not m:
            i += 1
            continue
        anchors = [m.group(1)]
        j = i + 1
        while j < n:
            mm = re.match(r"^\.\. _([a-z0-9_\-]+):\s*$", lines[j])
            if mm:
                anchors.append(mm.group(1))
            elif lines[j].startswith(".. "):  # a directive (video, figure) before the attribute
                j += 1
                while j < n and (lines[j].startswith("   ") or not lines[j].strip()):
                    j += 1
                continue
            elif lines[j].strip():
                break
            j += 1
        if j >= n or not lines[j].startswith(":at:"):
            i = j
            continue
        header = lines[j]
        k = j + 1
        para = []
        while k < n and (lines[k].startswith("   ") or not lines[k].strip()):
            if lines[k].strip().startswith(".. "):  # figures, admonitions
                k += 1
                while k < n and (lines[k].startswith("      ") or not lines[k].strip()):
                    k += 1
                continue
            line = lines[k].strip()
            if line.startswith("+"):  # grid table border
                k += 1
                continue
            if line.startswith("|") and line.endswith("|"):  # grid table row -> "a - b."
                cells = [c.strip() for c in line.strip("|").split("|")]
                line = " - ".join(c for c in cells if c) + ";"
            para.append(line)
            k += 1
        text = clean(" ".join(para))
        mval = re.search(r":at-val:`([^`]*)`", header)
        val = clean(mval.group(1)) if mval else ""
        if len(text) > 700:
            cut = text[:700]
            text = cut[: cut.rfind(". ") + 1] if ". " in cut else cut + "..."
        full = (f"[{val}] " if val else "") + text
        for a in anchors:
            if any(s in a for s in SKIP_SUBSTR):
                continue
            for pre, key in PREFIXES.items():
                if a.startswith(pre):
                    attr = a[len(pre):]
                    if "-" in attr:
                        break
                    docs.setdefault(key + attr, full)
                    break
        i = k


def write(docs: dict) -> None:
    out = sys.stdout
    out.write("// Generated by tools/gen_mjcf_docs.py from MuJoCo's documentation. Do not edit.\n")
    out.write("// Excerpts from the MuJoCo documentation, Copyright DeepMind Technologies Limited,\n")
    out.write("// licensed under the Apache License, Version 2.0.\n")
    out.write('#include "ui/MjcfDocs.h"\n\n#include <cstring>\n\nnamespace mjf {\n\nnamespace {\n')
    out.write("struct DocEntry {\n  const char* key;\n  const char* text;\n};\n")
    out.write("const DocEntry kDocs[] = {\n")
    for key in sorted(docs):
        t = docs[key].replace("\\", "\\\\").replace('"', '\\"')
        out.write(f'    {{"{key}", "{t}"}},\n')
    out.write("};\n}  // namespace\n\n")
    out.write(
        "const char* mjcfDoc(const char* key) {\n"
        "  size_t lo = 0, hi = sizeof(kDocs) / sizeof(kDocs[0]);\n"
        "  while (lo < hi) {\n"
        "    size_t mid = (lo + hi) / 2;\n"
        "    int c = std::strcmp(kDocs[mid].key, key);\n"
        "    if (c == 0) return kDocs[mid].text;\n"
        "    if (c < 0) lo = mid + 1; else hi = mid;\n"
        "  }\n"
        "  return nullptr;\n"
        "}\n\n}  // namespace mjf\n"
    )
    print(f"{len(docs)} entries", file=sys.stderr)


if __name__ == "__main__":
    main(sys.argv[1:])
