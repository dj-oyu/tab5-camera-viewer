import os
import shutil
import subprocess
Import("env")

# Run web build
print("Building Hono web app...")
root_dir = os.path.join(env.get("PROJECT_DIR"), ".")
subprocess.run(["echo", "hello"], cwd=root_dir, shell=True, check=True)

# Prune LVGL assembly if present to avoid RISC-V linking issues
# This is a workaround for LVGL 9 assembly files causing ABI mismatch in PIO
libdeps_dir = os.path.join(env.get("PROJECT_DIR"), ".pio", "libdeps", env.get("PIOENV"))
lvgl_blend_sw = os.path.join(libdeps_dir, "lvgl", "src", "draw", "sw", "blend")
if os.path.exists(lvgl_blend_sw):
    for folder in ["helium", "neon"]:
        p = os.path.join(lvgl_blend_sw, folder)
        if os.path.exists(p):
            print(f"Pruning LVGL folder: {p}")
            shutil.rmtree(p)

# Try to load .env file
env_file = os.path.join(env.get("PROJECT_DIR"), ".env")

if os.path.exists(env_file):
    print(f"Loading environment from {env_file}")
    with open(env_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            key, value = line.split("=", 1)
            # Remove quotes if present
            value = value.strip('"').strip("'")
            # Inject as build flag
            env.Append(CPPDEFINES=[
                (key, f'\\"{value}\\"')
            ])
else:
    print("Warning: .env file not found!")
