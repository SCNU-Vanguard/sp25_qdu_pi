# Workspace delivery rules

- Use the PC checkout as the source of truth; manage changes with Git on the PC only.
- Preserve original SP25 code and tools when usable. Prefer minimal adaptations to parallel implementations.
- Temporary probes, test harnesses, generated fixtures, and diagnostic downloads belong under ignored `build/`. Remove newly introduced one-off test code from the main source tree before delivery. Do not delete existing project tests or runtime helpers merely because their names or paths contain `test`.
- Before delivering a main-source sync, review and commit all main-source changes that must be included. Never discard existing user edits to make a package clean.
- Export the complete managed main source from one Git commit, including configs, CMake files, and required runtime headers. Provide a manifest to check content and unexpected source files on the Pi; a partial update is not proof that the two trees match.
- Exclude Git history, docs, build outputs, logs, user pictures, and calibration captures from main-source delivery. Put transfer/verification tooling under `build/`.
- Do not generate source backups on the Pi. Remove temporary transfer packages after successful deployment. Recover versions through PC Git and resync.
- Changes made during Pi-side debugging must be brought back to the PC and committed before the next source sync.
- Explain each proposed change and user command in plain Chinese: where to run it, why it is needed, what it changes, and what success looks like. Give small steps rather than unexplained command blocks.
- Do not claim the Pi matches the PC until the Pi-side verification has actually passed. Never delete unexpected or third-party Pi files merely to make a check pass.
