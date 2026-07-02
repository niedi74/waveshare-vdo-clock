Import("env")
import datetime
import subprocess

# Aktuelle lokale PC-Zeit beim Kompilieren als Build-Flags injizieren.
# Wird genutzt um den PCF85063 RTC beim Flashen auf die korrekte
# Wall-Clock-Zeit zu setzen. Bei jedem Build neu -> nach jedem Flash
# stimmt die Uhr (Genauigkeit ~ Compile+Flash-Dauer, ~30s).
now = datetime.datetime.now()
# Eindeutige Build-ID (Epoch) damit die Firmware erkennt "neuer Flash"
# und den RTC genau einmal pro Flash neu stellt.
build_id = int(now.timestamp())

env.Append(CPPDEFINES=[
    ("RTC_BUILD_Y",  now.year),
    ("RTC_BUILD_MO", now.month),
    ("RTC_BUILD_D",  now.day),
    ("RTC_BUILD_H",  now.hour),
    ("RTC_BUILD_MI", now.minute),
    ("RTC_BUILD_S",  now.second),
    ("RTC_BUILD_DOW", (now.weekday() + 1) % 7),  # Mo=0..So=6 -> So=0..Sa=6
    ("RTC_BUILD_ID", build_id),
])
print(f"[inject_time] RTC build time = {now.isoformat()} (id {build_id})")

# Git-Kurzhash injizieren: WebGUI zeigt ihn + verlinkt auf den GitHub-Stand.
# "+"-Suffix = Working Tree hatte beim Build uncommittete Aenderungen.
try:
    _cwd = env.subst("$PROJECT_DIR")
    rev = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"],
                                  cwd=_cwd, text=True).strip()
    if subprocess.call(["git", "diff", "--quiet"], cwd=_cwd) != 0:
        rev += "+"
except Exception:
    rev = "unknown"
env.Append(CPPDEFINES=[("GIT_REV", env.StringifyMacro(rev))])
print(f"[inject_time] GIT_REV = {rev}")
