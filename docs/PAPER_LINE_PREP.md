# Paper-Line Prep

This branch is for the paper-line only. Keep Ubuntu mainline changes out of scope.

Active working guard:

- `.codex/paper_line_workflow_guard.md` inside this repository.

## Baseline skills

Keep these ready for day-one work:

- `pdf` for PDF reading and extraction.
- `paper-reading` for structured paper summaries.
- `idea-generation` for narrowing the route into concrete candidate directions.
- `experiment-design` for baselines, ablations, and metrics.
- `paper-review` for reviewer-style criticism.
- `paper-writing` for outlines, section drafts, and revisions.
- `matlab-list-products` for checking installed MATLAB products.
- `matlab-debugging` for runtime diagnosis.
- `matlab-review-code` for code review and release-sensitive checks.
- `matlab-testing` for unit tests and regression checks.
- `matlab-analyze-data` for tables, time series, and result inspection.

Keep `matlab-agentic-toolkit-setup` around as a maintenance skill only. The MATLAB MCP setup is already done and should not be repeated unless the install changes.

## Later skills

Add these only when the work actually needs them:

- `matlab-optimize-performance`
- `matlab-write-performance-tests`
- `matlab-create-live-script` only if MATLAB is R2025a or newer.
- `proof-writer` only if the paper grows formal derivations.
- Simulink-specific skills only when Simulink becomes an active workstream.
- `academic-research` only if we want one umbrella entry instead of the narrower paper skills above.

## MATLAB setup path

- Keep the MathWorks toolkit clone at `C:/Users/wysxgd/tools/matlab-agentic-toolkit`.
- Do not write global Codex MCP config again unless the MATLAB install changes.
- Keep all research outputs under `research/`.

## Canonical route and evidence

- `docs/PAPER_LINE_ROUTE_BOOK_CN.md`
- `docs/PAPER_LINE_TECHNICAL_ROUTE.md`
- `docs/PAPER_LINE_PATENT_PREP.md`
- `docs/PAPER_LINE_MATLAB_EXPERIMENT_PLAN_CN.md`
- `docs/PAPER_LINE_ROS1_STAGE2_ADAPTER.md`
- `docs/PAPER_LINE_UBUNTU20_HANDOFF.md`
- `research/notes/2026-05-22_rebuilt_research_dossier.md`
- `research/notes/2026-05-22_chatgpt_rebuild_review.md`
- `research/notes/2026-05-22_rebuild_scope_log.md`
- `research/notes/2026-05-23_stress_suite_log.md`
- `research/notes/2026-05-23_ros1_stage2_adapter_log.md`
- `research/notes/2026-05-31_windows_matlab_refresh_log.md`
- Read this before making any implementation choice.

Historical context only:

- `docs/WINDOWS_PAPER_LINE_HANDOFF.md`

Do not treat the historical handoff as the final route authority if it differs
from the rebuilt route book.

## Prompt pack

Use the reusable prompts under `research/prompts/`:

- `00_route_convergence.md`
- `05_direction_screening.md`
- `10_paper_intake.md`
- `20_experiment_design.md`
- `30_matlab_scaffold.md`
- `40_matlab_review.md`
- `50_paper_outline.md`

## What not to do yet

- Do not touch `ubuntu-mainline`.
- Do not make Simulink the first dependency of the research loop.
- Do not add broad skills unless we need them for an active task.
- Do not make the ROS1 runtime depend on MATLAB.
- Do not convert this ROS1/catkin project to ROS2.
