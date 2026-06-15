# Windows command-line length workaround.
#
# NimBLE-Arduino compiles to ~190 object files. With `lib_archive = no` the
# full object list is passed straight to `ld` on the final link, and the
# combined command line overflows Windows' ~32 KB limit. The symptom is a
# non-deterministic "ld.exe: cannot find <some>.o: No such file or directory"
# (a different object each build) because the command line is silently
# truncated.
#
# Fix: wrap the long link/archive commands with SCons' TEMPFILE so the
# arguments are spilled into an @response-file that the GNU tools read.
Import("env")

for _name in ("LINKCOM", "ARCOM"):
    _cmd = env.get(_name)
    if _cmd and "TEMPFILE" not in _cmd:
        env[_name] = "${TEMPFILE('%s','$%sSTR')}" % (_cmd, _name)

# Spill to a response file whenever the command is long (default is generous
# on Windows, so force a conservative threshold).
env["MAXLINELENGTH"] = 8000
