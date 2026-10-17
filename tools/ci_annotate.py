"""Turns compiler errors and failed checks in a log into GitHub Actions error annotations.

Usage: python tools/ci_annotate.py build.log "Build"

Annotations show up on the run summary and through the API, which helps when the raw job log is
not at hand. GitHub keeps at most 10 error annotations per step, so errors are grouped.
"""
import re
import sys


def main():
  path, title = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "Error"
  with open(path, encoding="utf-8", errors="replace") as f:
    lines = f.read().splitlines()
  pattern = re.compile(r"\b(error|ERROR|Error)\b[ :]|\bFAILED\b|fatal error|undefined reference|Undefined symbols")
  hits = []
  for i, line in enumerate(lines):
    if pattern.search(line):
      hits.append("\n".join(lines[i:i + 4]))
    if len(hits) >= 50:
      break
  if not hits:
    hits = ["\n".join(lines[-30:])]  # no recognizable error: show the end of the log
  for start in range(0, len(hits), 5):
    msg = "\n\n".join(hits[start:start + 5])
    msg = msg.replace("%", "%25").replace("\r", "").replace("\n", "%0A")
    print(f"::error title={title}::{msg}")


if __name__ == "__main__":
  main()
