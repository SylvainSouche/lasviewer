# lasviewer — bmake-it workspace (https://github.com/SylvainSouche/bmake-it).
# bmake must find bmake-it's mk/ directory: run bmake-it's
# scripts/install-env.sh once, or set MAKESYSPATH=<bmake-it>/mk:<default>.
PARENT_WS=
.include "mk.workspace.mk"

# Formatting and naming check of the project's sources (scripts/lint.sh;
# needs a prior build). `bmake lint-fix` applies the fixes.
lint: .PHONY
	@sh ${.CURDIR}/scripts/lint.sh
lint-fix: .PHONY
	@sh ${.CURDIR}/scripts/lint.sh --fix
