"""Probe ESP32-P4 PIE instructions by trying to assemble them."""
import subprocess, tempfile, os, sys

AS = os.path.expanduser("~/.platformio/packages/toolchain-riscv32-esp/bin/riscv32-esp-elf-as.exe")

candidates = [
    # Arithmetic
    "esp.vadd.u32 q0, q1, q2",
    "esp.vadd.s32 q0, q1, q2",
    "esp.vsub.u32 q0, q1, q2",
    "esp.vsub.s32 q0, q1, q2",
    # Bitwise
    "esp.xorq q0, q1, q2",
    "esp.orq q0, q1, q2",
    "esp.andq q0, q1, q2",
    "esp.notq q0, q1",
    "esp.vxor q0, q1, q2",
    "esp.vor q0, q1, q2",
    "esp.vand q0, q1, q2",
    # Shift - register operand
    "esp.vsl.32 q0, q1, q2",
    "esp.vsr.u32 q0, q1, q2",
    "esp.vsr.s32 q0, q1, q2",
    "esp.vsll.32 q0, q1, q2",
    "esp.vsrl.32 q0, q1, q2",
    "esp.vsra.32 q0, q1, q2",
    # Shift - immediate operand
    "esp.vsl.32 q0, q1, 16",
    "esp.vsr.u32 q0, q1, 16",
    "esp.vsll.32 q0, q1, 16",
    "esp.vsrl.32 q0, q1, 16",
    "esp.vsra.32 q0, q1, 16",
    # Cross-lane byte shift (element permutation)
    "esp.vsld.8 q0, q1, q2, 4",
    "esp.vsrd.8 q0, q1, q2, 4",
    "esp.vsldi.8 q0, q1, 4",
    # Shuffle / zip
    "esp.vzip.8 q0, q1",
    "esp.vzip.16 q0, q1",
    "esp.vzip.32 q0, q1",
    "esp.vunzip.8 q0, q1",
    "esp.vunzip.16 q0, q1",
    "esp.vunzip.32 q0, q1",
    # Move Q register
    "esp.vmov.q q0, q1",
    "esp.mov.q q0, q1",
    # Load/Store
    "esp.vld.128.ip q0, a0, 16",
    "esp.vst.128.ip q0, a0, 16",
    "esp.vld.128.xp q0, a0, a1",
    "esp.vst.128.xp q0, a0, a1",
    # Hardware loop
    "esp.lp.setup 0, a0, .Lend",
    "esp.lp.setupi 0, 10, .Lend",
    # SAR (shift amount register)
    "esp.movx.w.sar a0",
    "esp.movx.r.sar a0",
    # Compare/Min/Max
    "esp.vcmp.eq.s32 q0, q1, q2",
    "esp.vmin.s32 q0, q1, q2",
    "esp.vmax.s32 q0, q1, q2",
    "esp.vmin.u32 q0, q1, q2",
    "esp.vmax.u32 q0, q1, q2",
    # Multiply
    "esp.vmul.u32 q0, q1, q2",
    "esp.vmul.s32 q0, q1, q2",
    # Additional shift variants
    "esp.vsl.32.r q0, q1",
    "esp.vsr.u32.r q0, q1",
    # Src2 immediate shift
    "esp.vsl.32.s q0, q1",
    "esp.vsr.u32.s q0, q1",
    # SRC instruction (concat+shift like S3)
    "esp.src.q q0, q1, q2",
    "esp.src.q.qup q0, q1, q2",
    "esp.src.q.ld.ip q0, q1, a0, 16",
]

valid = []
invalid = []
for instr in candidates:
    tmpfile = os.path.join(tempfile.gettempdir(), "pie_probe.S")
    tmpout = os.path.join(tempfile.gettempdir(), "pie_probe.o")
    with open(tmpfile, 'w') as f:
        f.write(f".text\n.Lstart:\n    {instr}\n.Lend:\n    nop\n")
    try:
        result = subprocess.run(
            [AS, "-march=rv32imafc_zicsr_xesppie", "-o", tmpout, tmpfile],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            valid.append(instr)
        else:
            err = result.stderr.strip().split('\n')[-1][:100]
            invalid.append((instr, err))
    except Exception as e:
        invalid.append((instr, str(e)[:60]))

try:
    os.unlink(tmpfile)
    os.unlink(tmpout)
except:
    pass

print("=== VALID PIE INSTRUCTIONS ===")
for v in valid:
    print(f"  {v}")
print(f"\nValid: {len(valid)}, Invalid: {len(invalid)}")
print(f"\n=== INVALID ===")
for name, err in invalid:
    print(f"  {name}")
    print(f"    -> {err}")
