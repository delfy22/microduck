# Development rules

## Design

- Always aim for the simple solution. Build an understandable and maintainable
  codebase that can be extended later, rather than covering all possibilities
  today.
- Prefer refactoring existing code towards a simpler or better solution over
  continually extending it with additional layers, options, or special cases.
- Keep changes focused. Avoid unrelated refactoring, formatting, or dependency
  updates.
- Keep dependencies deliberate. Reuse existing facilities before adding another
  dependency or abstraction.
- Prefer C++ implementations over Python, particularly for ROS nodes.

## Commits and validation

- Commit work in small, logical units, each representing one coherent change.
- Complete the linting and testing appropriate to each change before committing.
  All required checks must pass. If a required check fails or cannot be run,
  report the blocker and leave the work uncommitted until the checks pass.
  Do not treat an unrun check as a pass.
- Test observable behaviour and failure cases. Favour these over tests tied to
  implementation details.

## Git operations

- Leave pushing to the user.
- Do not rebase, amend, squash, or otherwise edit previous commits without
  explicit permission. Offer these operations when useful, but do not assume
  they are authorised.

## Language

- Use British English exclusively in code, comments, documentation, commit
  messages, and communication, unless a third-party API requires an alternative.
  Preserve names and spellings required by those APIs.

## Code documentation

- Use comments and docstrings to explain complex implementations and function
  and class interfaces, including relevant inputs, outputs, units, and side
  effects.
- Describe the current implementation. Do not refer to previous implementations
  or narrate the history of changes in code comments or docstrings.
- Document robotics conventions explicitly at interfaces: units, coordinate
  frames, timestamp sources, and command timeouts.
