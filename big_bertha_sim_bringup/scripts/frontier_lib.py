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

"""Frontier detection on an occupancy grid. ROS-free so it can be unit-tested."""

from collections import deque

FREE_MAX = 25  # occupancy <= this is free enough to stand on
OCC_MIN = 65   # occupancy >= this is a wall
UNKNOWN = -1


def has_clearance(data, width, height, index, radius):
    """
    Report whether no occupied cell lies within `radius` cells of `index`.

    A frontier cell is free but often sits right against the wall that hides
    the unknown space behind it. Standing the robot there puts it inside the
    costmap inflation, where the controller refuses to move and grinds until
    the goal times out. Requiring clearance keeps goals somewhere the robot
    can actually stand.
    """
    x0, y0 = index % width, index // width
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            x, y = x0 + dx, y0 + dy
            if 0 <= x < width and 0 <= y < height and data[y * width + x] >= OCC_MIN:
                return False
    return True


def find_frontiers(data, width, height, min_size, min_clear=0):
    """
    Cluster frontier cells in an occupancy grid.

    A frontier cell is free and 4-adjacent to unknown space -- the edge of
    what the robot has seen. Clusters are 8-connected; each is returned as
    (representative_cell_index, size). The representative is the cluster cell
    nearest the centroid, so a goal built from it always lands on a known-free
    cell: a raw centroid can fall inside the unknown region the arc wraps
    around, which the planner would refuse.
    """
    is_frontier = bytearray(width * height)
    for i, v in enumerate(data):
        if v < 0 or v > FREE_MAX:
            continue
        x, y = i % width, i // width
        if ((x > 0 and data[i - 1] == UNKNOWN)
                or (x < width - 1 and data[i + 1] == UNKNOWN)
                or (y > 0 and data[i - width] == UNKNOWN)
                or (y < height - 1 and data[i + width] == UNKNOWN)):
            is_frontier[i] = 1

    clusters = []
    seen = bytearray(width * height)
    for start in range(width * height):
        if not is_frontier[start] or seen[start]:
            continue
        cells = []
        queue = deque([start])
        seen[start] = 1
        while queue:
            i = queue.popleft()
            cells.append(i)
            x, y = i % width, i // width
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nx, ny = x + dx, y + dy
                    if not (0 <= nx < width and 0 <= ny < height):
                        continue
                    j = ny * width + nx
                    if is_frontier[j] and not seen[j]:
                        seen[j] = 1
                        queue.append(j)
        if len(cells) < min_size:
            continue
        cx = sum(c % width for c in cells) / len(cells)
        cy = sum(c // width for c in cells) / len(cells)
        ranked = sorted(
            cells, key=lambda c: (c % width - cx) ** 2 + (c // width - cy) ** 2)
        if min_clear > 0:
            # Nearest-the-centroid cell the robot can actually stand in. A
            # cluster with no such cell is hard against a wall -- skip it
            # rather than send a goal the controller will grind on.
            ranked = [c for c in ranked
                      if has_clearance(data, width, height, c, min_clear)]
            if not ranked:
                continue
        clusters.append((ranked[0], len(cells)))
    return clusters
