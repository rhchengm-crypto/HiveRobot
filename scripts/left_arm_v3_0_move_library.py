#!/usr/bin/env python3
"""v3.0 move adapters for pieces detected and grasped at C4.

All regular commands delegate to v2.8 unchanged.  The one v3.0 composition
uses the trained C4 grasp/clearance anchor and the existing bishop C1
destination move.
"""
from __future__ import annotations

import sys

import left_arm_v2_8_move_library as v28

WHITE_ROOK_PLACE_A1_MOVE_NAME = "white_rook_place_A1"


def build_parser():
    parser = v28.build_parser()
    subparsers = next(
        action for action in parser._actions
        if isinstance(action, v28.argparse._SubParsersAction)
    )
    subparsers.choices["replay-move"].add_argument(
        "--white-rook-place-a1",
        action="store_true",
        help="grasp at C4, use C4 clearance, place with white_rook_place_A1, then open claw",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    normalized = ""
    if args.cmd == "replay-move":
        import left_arm_v2_6_move_library as legacy
        normalized = legacy.normalize_move_name(args.name)
    c4_bishop = (
        args.cmd == "replay-move"
        and args.white_bishop_placement
        and normalized == v28.PLACEMENT_C4_MOVE_NAME
    )
    c4_rook = (
        args.cmd == "replay-move"
        and args.white_rook_place_a1
        and normalized == v28.PLACEMENT_C4_MOVE_NAME
    )
    if not (c4_bishop or c4_rook):
        return v28.main()

    # v2.8's White Bishop Placement is already the validated destination
    # sequence.  For v3.0 only, select the C4 grasp/clearance anchor as its
    # source instead of the historical D4 bishop01 source.
    original_source = v28.PLACEMENT1_MOVE_NAME
    original_destination = v28.WHITE_BISHOP_PLACE_MOVE_NAME
    original_place = v28.run_white_bishop_place_on_arm
    original_claw_home = v28.claw_home_while_holding_arm
    v28.PLACEMENT1_MOVE_NAME = v28.PLACEMENT_C4_MOVE_NAME
    if c4_rook:
        args.white_bishop_placement = True
        v28.WHITE_BISHOP_PLACE_MOVE_NAME = WHITE_ROOK_PLACE_A1_MOVE_NAME

        def rook_place(*call_args, **call_kwargs):
            call_kwargs["workflow_label"] = "White Rook Place_A1"
            return original_place(*call_args, **call_kwargs)

        def rook_claw_home(*call_args, **call_kwargs):
            call_kwargs["workflow_label"] = "White Rook Place_A1"
            return original_claw_home(*call_args, **call_kwargs)

        v28.run_white_bishop_place_on_arm = rook_place
        v28.claw_home_while_holding_arm = rook_claw_home
        print(
            "v3.0 C4 rook routing: white_knight_c4 grasp -> "
            "white_rook_place_A1 destination",
            flush=True,
        )
    try:
        v28.replay_with_local_bias(args)
    finally:
        v28.PLACEMENT1_MOVE_NAME = original_source
        v28.WHITE_BISHOP_PLACE_MOVE_NAME = original_destination
        v28.run_white_bishop_place_on_arm = original_place
        v28.claw_home_while_holding_arm = original_claw_home


if __name__ == "__main__":
    main()
