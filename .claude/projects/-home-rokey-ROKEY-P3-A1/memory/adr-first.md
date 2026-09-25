---
name: adr-first
description: Re-read docs/ADR/ADR_navigation2.md (and ADR_basic.md) before any Nav2/docking change or guidance; user flagged that I stopped consulting it
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 1e88eef7-43a9-4393-aade-53dea58994dd
  modified: 2026-09-23T23:25:52.474Z
---

Before touching anything Nav2/docking-related or writing a guidance, open `/home/rokey/ROKEY_P3_A1/docs/ADR/ADR_navigation2.md` and check the change against sections 2.1–2.7. Do the same for `ADR_basic.md`.

**Why:** On 2026-09-24 the user said I was not only breaking ADR items but had stopped reading the file at all. Concrete misses: `feeder_dock` started while Nav2 was still decelerating (ADR 2.2: dock only when Nav2 quiet ≥ 2 s); guidance used `--once` where ADR 2.3 says `-t 3 -r 1`; sim results were reported like field results (ADR 2.7).

**How to apply:** Read the ADR at the start of each Nav2 task and again before writing guidance. The ADR is not gospel: it lags behind. When it conflicts with something the user and I agreed in recent conversation (e.g. standoff 0.85 vs ADR 0.90), the recent agreement wins — and I must then propose the ADR update to the user (list the stale items in the report). Never edit the ADR myself without instruction or approval. Related: [[nav2-track-v008]], [[guidance-doc-conventions]].
