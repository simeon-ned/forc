# Release checklist

This edition is a teaching draft, not a published or peer-reviewed course release.

- Review the 12 lecture notes, worked examples, slide decks, and assessment rubrics.
- Confirm dates, grading policies, prerequisites, and optional topics with the instructor.
- Run uv sync --locked, uv run --locked pytest, and uv run --locked python scripts/run_practices.py.
- Build with npm --prefix site ci and npm --prefix site run build, then run npm --prefix site run check.
- Check both the root-hosted preview and the intended production path prefix.
- Verify external links and source licenses. The playlist links are companion material; full video transcripts have not been reviewed or copied.
- Keep generated datasets, environments, and temporary output out of the source commit.
- Review and commit only intended changes, preserving unrelated work.
- Set a release tag and update version metadata together when approved. CITATION.cff currently has no DOI or release date.
- The Pages workflow publishes only `site/_site` on a `main`/`master` push or manual trigger, after numerical practices and generated links pass.

No repository push, release tag, deployment, or DOI registration was performed while creating this draft.
