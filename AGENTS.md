# Repository working agreements

- Read README.md, PROJECT_STATUS.md and docs/LIMITATIONS.md before changes.
- Code assets are selected historical engineering work; public documentation may also describe the user's explicitly approved high-level direction.
- The current first-stage focus is active human participation and cooperative assistance, with local risk prediction and impact attenuation as core requirements. Keep planned capabilities separate from implemented evidence.
- The selected application is lower-limb level walking and stair-ascent assistance for older adults. Protection mechanisms remain replaceable; do not treat a candidate actuator as a frozen requirement.
- The initial target users can still walk independently but tire easily or struggle with stair ascent. Preserve active human participation; specific population parameters remain open.
- Preserve historical datasets; use a new versioned file for new evidence.
- Keep control units, timing, command ownership and failure behavior explicit.
- Run `python scripts/check_repository.py`; run the existing offline verification for code changes.
- Software tests do not certify ROS 2/Webots motion, mechanical safety or measured energy savings.
- Do not add credentials, personal learning records, private project documents or unapproved external materials.
- Keep changes scoped; do not start simulations, install tools or change hardware as part of repository maintenance.
