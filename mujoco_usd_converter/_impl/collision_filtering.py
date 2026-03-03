# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

import operator
from functools import reduce

import mujoco
import usdex.core
from pxr import Gf, Tf, Usd, UsdGeom, UsdPhysics, Vt

from .data import ConversionData, Tokens

__all__ = ["convert_collision_filtering"]


def convert_collision_filtering(data: ConversionData):
    if not data.geom_collision_filtering:
        return

    physics_scope = data.content[Tokens.Physics].GetDefaultPrim().GetChild(Tokens.Physics)

    # Create a dict per contype for collision filtering.
    collision_group: dict[int, [str]] = {}
    for geom_name, (contype, conaffinity) in data.geom_collision_filtering.items():
        print(f"[{geom_name}] collision_filtering: {contype}, {conaffinity}")
        if contype not in collision_group:
            collision_group[contype] = []
        collision_group[contype].append(geom_name)

    print(f"collision_group: {collision_group}")

    stage = physics_scope.GetStage()
    print(f"physics_path: {physics_scope.GetPath()}")
    for contype in collision_group:
        # Get the contype shift amount.
        index = (contype.bit_length() - 1) if contype else 0

        geom_names = collision_group[contype]
        conaffinity_list = [data.geom_collision_filtering[geom_name][1] for geom_name in geom_names]

        # Compute the bitwise OR of all values in conaffinity_list.
        conaffinity_or_result = reduce(operator.or_, conaffinity_list, 0)

        print(f"conaffinity_list: {conaffinity_list}")
        print(f"conaffinity_or_result: {conaffinity_or_result} -> {contype & conaffinity_or_result}")

        prim_path = physics_scope.GetPath().AppendChild(f"CollisionGroup_{index}")
        collision_group_prim = UsdPhysics.CollisionGroup.Define(stage, prim_path)

        # Add geom paths that belong to a Collision Group.
        collision_api = collision_group_prim.GetCollidersCollectionAPI()
        include_rel = collision_api.GetIncludesRel()
        for geom_name in geom_names:
            geom_path = data.geom_targets[geom_name]
            include_rel.AddTarget(geom_path)

        filtered_groups_rel = collision_group_prim.GetFilteredGroupsRel()
        if not (contype & conaffinity_or_result):
            filtered_groups_rel.AddTarget(collision_group_prim.GetPath())
