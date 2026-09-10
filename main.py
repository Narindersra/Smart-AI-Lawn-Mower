from pathlib import Path
import subprocess
import sys


# ============================================================
# PROJECT PATHS
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parent

WORLD_FILE = (
    PROJECT_ROOT
    / "simulation"
    / "worlds"
    / "lawn_world.wbt"
)


# ============================================================
# WEBOTS EXECUTABLE
# ============================================================

WEBOTS_EXE = (
    Path.home()
    / "AppData"
    / "Local"
    / "Programs"
    / "Webots"
    / "msys64"
    / "mingw64"
    / "bin"
    / "webots.exe"
)


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print("=" * 70)
    print("SMART AI LAWN MOWER - WEBOTS LAUNCHER")
    print("=" * 70)

    print()
    print("PROJECT ROOT:")
    print(PROJECT_ROOT)

    print()
    print("WORLD FILE:")
    print(WORLD_FILE)

    print()
    print("WEBOTS EXE:")
    print(WEBOTS_EXE)

    # --------------------------------------------------------
    # Check project
    # --------------------------------------------------------

    if not PROJECT_ROOT.exists():
        print()
        print("ERROR: Project root not found.")
        sys.exit(1)

    # --------------------------------------------------------
    # Check world
    # --------------------------------------------------------

    if not WORLD_FILE.exists():
        print()
        print("ERROR: lawn_world.wbt was not found.")
        print()
        print("Expected location:")
        print(WORLD_FILE)
        sys.exit(1)

    # --------------------------------------------------------
    # Check Webots
    # --------------------------------------------------------

    if not WEBOTS_EXE.exists():
        print()
        print("ERROR: Webots executable was not found.")
        print()
        print("Expected location:")
        print(WEBOTS_EXE)
        sys.exit(1)

    # --------------------------------------------------------
    # Check world header
    # --------------------------------------------------------

    print()
    print("--- WORLD CHECK ---")

    try:
        with open(WORLD_FILE, "r", encoding="utf-8-sig") as file:
            first_line = file.readline().rstrip()
    except OSError as error:
        print("ERROR: Could not read world file.")
        print(error)
        sys.exit(1)

    print("World header:", repr(first_line))

    if first_line != "#VRML_SIM R2025a utf8":
        print()
        print("ERROR: Invalid Webots world header.")
        print()
        print("Expected:")
        print("#VRML_SIM R2025a utf8")
        sys.exit(1)

    # --------------------------------------------------------
    # Everything is ready
    # --------------------------------------------------------

    print()
    print("Project root : OK")
    print("World file   : OK")
    print("Webots       : OK")
    print("World header : OK")

    # --------------------------------------------------------
    # Launch Webots
    # --------------------------------------------------------

    command = [
        str(WEBOTS_EXE),
        str(WORLD_FILE)
    ]

    print()
    print("--- LAUNCHING WEBOTS ---")
    print()
    print("Command:")
    print(" ".join(f'"{item}"' for item in command))
    print()

    try:
        subprocess.Popen(
            command,
            cwd=str(PROJECT_ROOT)
        )
    except OSError as error:
        print("ERROR: Failed to start Webots.")
        print(error)
        sys.exit(1)

    print("Webots launched successfully.")
    print("Opening:")
    print(WORLD_FILE)
    print()


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()