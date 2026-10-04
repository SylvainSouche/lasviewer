#!/usr/bin/env atf-sh
# DEM rendering, checked on real snapshots (lasviewer --snapshot) of the
# synthetic DEMs written by render_tool. A snapshot opens a window briefly.

# macOS kills (SIGKILL, exit 137) a program whose file was overwritten in
# place after an earlier run: bmake-it's copy-up does that to bin/lasviewer
# on a rebuild (reported). Say so instead of failing obscurely.
check_lasviewer_starts() {
    lasviewer -h > /dev/null 2>&1
    case $? in
    0) ;;
    137) atf_fail "macOS killed lasviewer at start (exit 137): bin/lasviewer was overwritten in place by a rebuild; delete build/*/bin/lasviewer and Viewer/build/*/bin/lasviewer, then bmake" ;;
    *) atf_fail "lasviewer -h failed" ;;
    esac
}

# Seen from below the terrain, any background pixel enclosed by surface is a
# hole through the mesh. A 20° collapse angle gives large coarse patches and
# many one-level transitions, where cracks would appear (they did with the
# old 1:1 transition rule: docs/design-tessellation-displacement.md §6v).
atf_test_case crack_free
crack_free_head() {
    atf_set descr "the DEM mesh has no holes at level transitions (seen from below)"
    atf_set timeout 600
}
crack_free_body() {
    check_lasviewer_starts
    atf_check -s exit:0 render_tool dem dem.tif terrain
    for view in 1000256,5999744,40,450,30,-35 1000150,5999650,50,250,120,-50 \
                1000350,5999800,60,300,250,-25 1000300,5999600,60,150,60,-40; do
        atf_check -s exit:0 -o ignore -e ignore \
            lasviewer -d dem.tif --dem-lod 20,6 --view "$view" --snapshot below.ppm
        cov=$(render_tool coverage below.ppm)
        holes=$(render_tool holes below.ppm)
        echo "view $view: coverage $cov, holes $holes"
        case "$cov" in 0.0*|0.1*) atf_fail "view $view shows too little surface ($cov)" ;; esac
        [ "$holes" -eq 0 ] || atf_fail "view $view: $holes background pixels show through the mesh"
    done
}

# Terrain + height model: loads, pairs, renders, exits cleanly.
atf_test_case mnt_mnh
mnt_mnh_head() {
    atf_set descr "an MNT with an MNH over it loads and renders"
    atf_set timeout 300
}
mnt_mnh_body() {
    check_lasviewer_starts
    atf_check -s exit:0 render_tool dem mnt.tif terrain
    atf_check -s exit:0 render_tool dem mnh.tif height
    atf_check -s exit:0 -o ignore -e match:"stands on" \
        lasviewer -mnt mnt.tif -mnh mnh.tif --snapshot both.ppm
    cov=$(render_tool coverage both.ppm)
    case "$cov" in 0.0*|0.1*) atf_fail "the scene shows too little surface ($cov)" ;; esac
}

# Several DEM tiles share one elevation colour ramp: one terrain loaded as a
# single file and as two half-tiles (the east half ~50 m higher) look the
# same from above. With a ramp per file they differed by ~37/255 (each half
# spanning violet to yellow on its own); with the shared ramp by < 0.1.
atf_test_case shared_colour_ramp
shared_colour_ramp_head() {
    atf_set descr "adjacent DEM tiles use one elevation colour ramp"
    atf_set timeout 300
}
shared_colour_ramp_body() {
    check_lasviewer_starts
    atf_check -s exit:0 render_tool dem full.tif slope
    atf_check -s exit:0 render_tool dem west.tif slope west
    atf_check -s exit:0 render_tool dem east.tif slope east
    view=1000256,5999744,150,700,0,89 # from above
    atf_check -s exit:0 -o ignore -e ignore lasviewer -d full.tif --view $view --snapshot one.ppm
    atf_check -s exit:0 -o ignore -e ignore \
        lasviewer -d west.tif -d east.tif --view $view --snapshot two.ppm
    diff=$(render_tool diff one.ppm two.ppm)
    echo "one file vs two tiles: mean difference $diff / 255"
    awk -v d="$diff" 'BEGIN { exit !(d >= 0 && d < 2.0) }' ||
        atf_fail "two tiles are coloured differently from one file (mean difference $diff / 255)"
}

atf_init_test_cases() {
    atf_add_test_case crack_free
    atf_add_test_case mnt_mnh
    atf_add_test_case shared_colour_ramp
}
