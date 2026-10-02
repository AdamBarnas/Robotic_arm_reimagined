"""Convert STL files to OBJ using FreeCAD's Mesh module.

Usage:
    python stl_to_obj.py stl/B2.stl              # writes stl/B2.obj
    python stl_to_obj.py stl/                    # converts every .stl in the folder
    python stl_to_obj.py stl/*.stl -o obj/       # writes into another folder
    python stl_to_obj.py stl/ --force            # overwrite existing .obj files

Coordinates and units are kept as in the STL (no recentring or scaling).
"""
import argparse
import os
import sys
from pathlib import Path

FREECAD_LIB = os.environ.get("FREECAD_LIB", "/usr/lib/freecad/lib")
sys.path.append(FREECAD_LIB)
try:
    import FreeCAD  # noqa: F401  (must be imported before Mesh)
    import Mesh
except ImportError:
    sys.exit(f"Cannot import FreeCAD from {FREECAD_LIB}; set FREECAD_LIB to your FreeCAD lib folder.")


def find_stl_files(inputs):
    for path in map(Path, inputs):
        if path.is_dir():
            yield from sorted(f for f in path.iterdir() if f.suffix.lower() == ".stl")
        elif path.suffix.lower() == ".stl":
            yield path
        else:
            print(f"skip {path}: not an .stl file or folder")


def main():
    parser = argparse.ArgumentParser(description="Convert STL files to OBJ using FreeCAD.")
    parser.add_argument("inputs", nargs="+", help=".stl files or folders containing them")
    parser.add_argument("-o", "--out-dir", type=Path, help="output folder (default: next to each input)")
    parser.add_argument("-f", "--force", action="store_true", help="overwrite existing .obj files")
    args = parser.parse_args()

    if args.out_dir:
        args.out_dir.mkdir(parents=True, exist_ok=True)

    for stl in find_stl_files(args.inputs):
        obj = (args.out_dir or stl.parent) / (stl.stem + ".obj")
        if obj.exists() and not args.force:
            print(f"skip {stl}: {obj} exists (use --force to overwrite)")
            continue
        mesh = Mesh.Mesh(str(stl))
        mesh.write(str(obj))
        print(f"{stl} -> {obj} ({mesh.CountFacets} facets)")


if __name__ == "__main__":
    main()
