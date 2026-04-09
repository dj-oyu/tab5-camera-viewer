"""Probe PIE shift/src instructions with various operand formats."""
import subprocess, tempfile, os

AS = os.path.expanduser("~/.platformio/packages/toolchain-riscv32-esp/bin/riscv32-esp-elf-as.exe")

candidates = [
    # Shift with 2 Q operands (SAR-based?)
    "esp.vsl.32 q0, q1",
    "esp.vsr.u32 q0, q1",
    "esp.vsr.s32 q0, q1",
    # Maybe the shift takes SAR implicitly
    "esp.movx.w.sar a0\n    esp.vsl.32 q0, q1",
    # src.q variants - concatenate + shift by SAR bytes
    "esp.src.q q0, q1, q2",
    "esp.src.q.qup q0, q1, q2",
    "esp.src.q.ld.ip q0, q1, a0, 0",
    "esp.src.q.ld.ip q1, q0, a0, 16",
    "esp.src.q.ld.xp q0, q1, a0, a1",
    "esp.src.q.qup.ld.ip q0, q1, a0, 16",
    # SAR byte variant
    "esp.movx.w.sar.bytes a0",
    "esp.movx.r.sar.bytes a0",
    # Try different shift mnemonics
    "esp.vrsl.32 q0, q1",
    "esp.vrsr.32 q0, q1",
    "esp.vrot.32 q0, q1",
    "esp.vrotl.32 q0, q1",
    # SLL/SRL per-element
    "esp.sll.s32 q0, q1",
    "esp.srl.s32 q0, q1",
    # Element extract/insert
    "esp.extr.s32 a0, q0, 0",
    "esp.ins.s32 q0, a0, 0",
    "esp.movi.q q0, a0, 0",
    # Possible 2-operand shift with immediate via different encoding
    "esp.vsl.32.s q0, q1",
    # Try esp.vsl with SAR as implicit
    "esp.vsl.32.sar q0, q1",
    # Additional src.q probing
    "esp.src.q.ld.ip q0, q2, a0, 16",
]

valid = []
invalid = []
tmpfile = os.path.join(tempfile.gettempdir(), "pie_probe2.S")
tmpout = os.path.join(tempfile.gettempdir(), "pie_probe2.o")

for instr in candidates:
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
            invalid.append((instr.replace('\n', ' ; '), err))
    except Exception as e:
        invalid.append((instr.replace('\n', ' ; '), str(e)[:60]))

try:
    os.unlink(tmpfile)
    os.unlink(tmpout)
except:
    pass

print("=== VALID ===")
for v in valid:
    print(f"  {v}")
print(f"\n=== INVALID ===")
for name, err in invalid:
    print(f"  {name}")
    print(f"    -> {err}")
