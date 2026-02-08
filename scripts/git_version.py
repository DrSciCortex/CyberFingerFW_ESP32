Import("env")
import subprocess

project_dir = env.subst("$PROJECT_DIR")

try:
    h = subprocess.check_output(
        ["git", "rev-parse", "--short", "HEAD"],
        cwd=project_dir,
        stderr=subprocess.STDOUT,
    ).decode("utf-8", errors="ignore").strip()
except Exception:
    h = "unknown"

# Inject as a *string literal* macro: -DGIT_COMMIT_HASH=\"abc123\"
env.Append(CPPDEFINES=[("GIT_COMMIT_HASH", h)])

print("GIT_COMMIT_HASH =", h)
