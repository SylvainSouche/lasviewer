# lasviewer — bmake-it workspace (https://github.com/SylvainSouche/bmake-it).
# bmake must find bmake-it's mk/ directory: run bmake-it's
# scripts/install-env.sh once, or set MAKESYSPATH=<bmake-it>/mk:<default>.
PARENT_WS=
.include "mk.workspace.mk"
