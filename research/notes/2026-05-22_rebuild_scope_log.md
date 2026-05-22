# 2026-05-22 Paper-Line Rebuild Scope Log

Scope: paper-line only.

## Why This Rebuild Exists

The previous route artifacts are being replaced because the user requires the
route to be rebuilt from an explicit workflow:

- live literature / patent / product / technical research
- RoxyBrowser + web ChatGPT collaboration
- evidence-backed decisions
- MATLAB-verifiable prototypes
- clean artifact hygiene

## Removed As Obsolete

These files were removed because they contained prior route conclusions or
review notes from the earlier incomplete workflow:

- `docs/PAPER_LINE_TECHNICAL_ROUTE.md`
- `research/notes/2026-05-22_chatgpt_route_review.md`
- `research/notes/2026-05-22_matlab_batch_experiments.md`
- `research/notes/2026-05-22_paper_line_research_dossier.md`

Generated outputs removed to prevent mixing old results into the rebuild:

- `research/outputs/paper_line_batch/`
- `research/figures_generated/paper_line_batch/`
- `research/figures_generated/paper_line_demo/`

## Preserved

The MATLAB prototype code under `research/matlab/` is preserved as a reusable
experimental tool, not as final route evidence. It can be rerun or changed after
the rebuilt route decisions are documented.

`docs/WINDOWS_PAPER_LINE_HANDOFF.md` is preserved as historical context only. It
is not the final route authority.

## Rebuild Deliverables

The rebuild will create:

- a source-backed research dossier
- a ChatGPT collaboration note with prompt and response summary
- a rebuilt technical route book
- updated MATLAB logs only after rerunning checks/experiments
