# UAV_Project — agent working notes

## Persistent memory (engram)

Session memory is filed under the project detected from the working directory
you launch `opencode` from. This file lives in this repo, so:

- Launch `opencode` from inside this directory (`cd ~/UAV_Project && opencode`)
  so this session's notes are filed under this project.
- If you launch opencode from another project's directory, notes file under
  that project instead. Project routing is per-session, never global.
- Do NOT set `ENGRAM_PROJECT` globally (e.g. in `~/.bashrc`) — that would pin
  *every* session to this project.
- If a save fails with an ambiguous-project error, use the CLI explicitly:
  `engram save "<title>" "<message>" --project UAV_Project`
  (the `--project` flag must come after the positional args).