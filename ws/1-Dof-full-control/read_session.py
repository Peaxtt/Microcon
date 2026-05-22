"""
Read Claude Code session log and extract conversation summary.
Usage:  python read_session.py
Output encoding: ascii-safe (works on any Windows terminal)
"""

import json, textwrap, re, os, glob, sys

# Force UTF-8 output on Windows
sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# ── Locate the JSONL file ──────────────────────────────────────────────────────
PROJECTS_DIR = r"C:\Users\Predator\.claude\projects"
PROJECT_KEY  = "C--PaYae-Microcon-ws-1-Dof-full-control"
project_dir  = os.path.join(PROJECTS_DIR, PROJECT_KEY)

jsonl_files  = glob.glob(os.path.join(project_dir, "*.jsonl"))
if not jsonl_files:
    print("No session file found in", project_dir)
    sys.exit(1)

SESSION_FILE = max(jsonl_files, key=os.path.getmtime)
print(f"Reading: {SESSION_FILE}")
print("="*70)

# ── Parse JSONL ───────────────────────────────────────────────────────────────
messages = []
with open(SESSION_FILE, encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue

        role    = obj.get("type")
        content = obj.get("message", {})

        if role == "user":
            parts = content.get("content", "")
            if isinstance(parts, list):
                text = " ".join(p.get("text","") for p in parts if isinstance(p, dict))
            else:
                text = str(parts)
            if text.strip():
                messages.append(("USER", text.strip()))

        elif role == "assistant":
            parts = content.get("content", [])
            if isinstance(parts, list):
                text = " ".join(
                    p.get("text","") for p in parts
                    if isinstance(p, dict) and p.get("type") == "text"
                )
            else:
                text = str(parts)
            if text.strip():
                messages.append(("ASST", text.strip()))

# ── Keyword filter ─────────────────────────────────────────────────────────────
BUG_KEYWORDS = re.compile(
    r"(bug|fix|error|wrong|corrupt|overflow|fail|crash|"
    r"heartbeat|modbus|wordlength|buffer|baud|parity|pid|dma|uart|"
    r"\bไม่ได้\b|\bผิด\b|\bแก้\b|\bหาย\b)",
    re.IGNORECASE
)

WRAP = 100
def wrap(text, prefix="  "):
    out = []
    for line in text.splitlines():
        if len(line) <= WRAP:
            out.append(prefix + line)
        else:
            for chunk in textwrap.wrap(line, WRAP):
                out.append(prefix + chunk)
    return "\n".join(out)

# ── Print all messages ─────────────────────────────────────────────────────────
print(f"\nTotal messages: {len(messages)}\n")
print("="*70)

for i, (role, text) in enumerate(messages):
    preview_lines = [l for l in text.splitlines() if l.strip()][:3]
    preview = " | ".join(preview_lines)[:200]
    flag = " [!]" if BUG_KEYWORDS.search(text) else "    "
    label = "[USER]" if role == "USER" else "[ASST]"
    print(f"\n[{i+1:03d}] {label}{flag}")
    print(wrap(preview))

# ── Bug/Fix summary ────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("\nBUG / FIX MESSAGES FROM ASSISTANT:\n")
for i, (role, text) in enumerate(messages):
    if BUG_KEYWORDS.search(text) and role == "ASST":
        sentences = re.split(r'[.!\?\n]', text)
        hits = [s.strip() for s in sentences if BUG_KEYWORDS.search(s) and len(s.strip()) > 20]
        if hits:
            print(f"[{i+1:03d}] ASST:")
            for h in hits[:4]:
                print(f"   -> {h[:115]}")
            print()

# ── Change log ─────────────────────────────────────────────────────────────────
print("="*70)
print("\nKEY CHANGES MADE IN THIS SESSION:\n")
changes = [
    ("MODBUS_REG_COUNT",    "50 -> 64  *** ROOT CAUSE of heartbeat fail (buffer overflow at 0x3F)"),
    ("WORDLENGTH fix",      "UART_WORDLENGTH_8B -> 9B  (7E1 -> 8E1, must match BaseSystem)"),
    ("PID gains",           "kp_vel 10->30, ki_vel 0.5->0.1, kd_vel 0.5->0.0 (control team vals)"),
    ("Trajectory params",   "a_max 6.28->12.56, t_acc 0.8->0.3, t_cruise 1.186->0.2"),
    ("New files",           "pid_control.h/.c  REF_FEEDFORWARD.h/.c  DistFF.h/.c"),
    ("pid_enabled",         "New — set 0 to freeze all PID + PWM=0 instantly"),
    ("refff_enabled",       "New — set 1 to enable model-based voltage feedforward"),
    ("motor_dir_inverted",  "New — set 1 to flip motor direction without rewiring"),
    ("hard_stop_active",    "New — set 1 to force PWM=0 for one tick"),
    ("STATE_SEQUENCE",      "Split from STATE_AUTO  pick/place sequence is its own state"),
    ("STATE_TEST",          "Split from STATE_AUTO  precision/perf test is its own state"),
    ("Set Home -> homed=1", "set_home now also sets homed=1  test P2P without sensor"),
    ("Tower lights",        "R = near-limit warning >440deg  EMER-released = all 3 blink"),
    ("DPAD fine tune",      "Uses fine_tune_speed variable  works in IDLE + MANUAL"),
    ("UART4 removed",       "Was causing linker error after IOC regen"),
]
for name, desc in changes:
    print(f"  [OK]  {name:<26} {desc}")

# ── Test procedure ─────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("\nHOW TO TEST RIGHT NOW:\n")
steps = [
    "STM32CubeIDE  Build  :  Ctrl+B  (or hammer icon, wait for 0 errors)",
    "STM32CubeIDE  Flash  :  F11     (or green Run button)",
    "Open BaseSystem backend:",
    "    docker-compose up -d               (C:\\PaYae\\Studio\\FRA263-264_BaseSystem)",
    "    .\\main_v1.1.exe                    (keep terminal open)",
    "    Browser -> http://localhost:3000",
    "BaseSystem UI:",
    "    COM port = COM14   (STMicroelectronics STLink Virtual COM Port)",
    "    Click Connect",
    "    Heartbeat should go GREEN",
    "Test control (no sensor needed):",
    "    BaseSystem -> Set Home    (zeros encoder + homed=1)",
    "    BaseSystem -> P2P -> target=90 -> Send",
    "    Motor should rotate 90 deg",
    "If motor direction wrong:",
    "    Live Expressions -> motor_dir_inverted = 1",
    "Enable feedforward (optional, tune later):",
    "    Live Expressions -> refff_enabled = 1",
    "    Live Expressions -> V_supply = 24.0  (your bus voltage)",
]
for s in steps:
    prefix = "  " if s.startswith(" ") else "  "
    print(prefix + s)

print()
