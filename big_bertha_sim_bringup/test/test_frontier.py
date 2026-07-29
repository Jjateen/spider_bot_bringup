# Copyright 2026 Jjateen Gundesha
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Frontier detection: the explorer's stop condition depends on it."""

import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))

from frontier_lib import find_frontiers, has_clearance

W = H = 8


def grid(fill):
    return [fill] * (W * H)


def test_fully_known_map_has_no_frontier():
    """The explorer's terminating condition: all free, nothing unknown."""
    assert find_frontiers(grid(0), W, H, 1) == []


def test_fully_unknown_map_has_no_frontier():
    """No free cell to stand on -> nothing to drive to (not 'done')."""
    assert find_frontiers(grid(-1), W, H, 1) == []


def test_free_unknown_boundary_is_a_frontier():
    """Left half free, right half unknown -> one cluster on the seam."""
    data = grid(-1)
    for y in range(H):
        for x in range(4):
            data[y * W + x] = 0
    clusters = find_frontiers(data, W, H, 1)
    assert len(clusters) == 1
    rep, size = clusters[0]
    assert size == H            # one frontier cell per row
    assert rep % W == 3         # the last free column


def test_occupied_cells_are_not_frontiers():
    """A wall against unknown space is not explorable -- only free cells are."""
    data = grid(-1)
    for y in range(H):
        data[y * W + 3] = 100
    assert find_frontiers(data, W, H, 1) == []


def test_min_size_filters_noise():
    """Single-cell specks must not keep exploration alive forever."""
    data = grid(-1)
    data[2 * W + 2] = 0
    assert len(find_frontiers(data, W, H, 1)) == 1
    assert find_frontiers(data, W, H, 2) == []


def test_separate_regions_cluster_separately():
    """Two disjoint openings -> two goals, not one averaged into a wall."""
    data = grid(100)
    for y in (1, 2):
        data[y * W + 1] = 0
        data[y * W + 1 - 1] = -1
    for y in (5, 6):
        data[y * W + 6] = 0
        data[y * W + 6 + 1] = -1
    assert len(find_frontiers(data, W, H, 1)) == 2


def test_clearance_rejects_cells_beside_a_wall():
    """The goal must not sit inside costmap inflation."""
    data = grid(0)
    data[4 * W + 4] = 100
    assert not has_clearance(data, W, H, 4 * W + 2, 2)  # 2 cells away
    assert has_clearance(data, W, H, 4 * W + 1, 2)      # 3 cells away


def test_clearance_filter_drops_wall_hugging_frontiers():
    """A frontier with no standable cell is skipped, not sent as a goal."""
    data = grid(-1)
    for y in range(H):          # free column beside...
        data[y * W + 4] = 0
    for y in range(H):          # ...a solid wall right next to it
        data[y * W + 5] = 100
    assert find_frontiers(data, W, H, 1, 0)       # found without the filter
    assert find_frontiers(data, W, H, 1, 3) == []  # dropped with it


def test_representative_is_a_free_cell():
    """A U-shaped arc's centroid falls in unknown space; the rep must not."""
    data = grid(-1)
    for x in range(1, 7):
        data[6 * W + x] = 0       # bottom of the U
    for y in range(2, 7):
        data[y * W + 1] = 0       # left arm
        data[y * W + 6] = 0       # right arm
    clusters = find_frontiers(data, W, H, 1)
    assert clusters
    for rep, _ in clusters:
        assert data[rep] == 0
