#!/usr/bin/env python3
"""Scene catalog for the free-play demo: which simulated worlds can be loaded,
and what is actually in each one.

The demo used to be pinned to one LIBERO benchmark task chosen on the command
line, so the audience could only sensibly ask for things that one world
contained. This module enumerates every installed LIBERO world plus the local
sandbox scenes in ``demo/scenes/``, parses each BDDL file for the objects and
fixtures it instantiates, and derives prompt suggestions from those objects.

A "scene" here is only a world: geometry, objects, initial layout. The benchmark
goal predicate that ships in the same BDDL file is deliberately *not* used to
score or terminate anything -- free play has no goal.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

# LIBERO asset names are not the words a person says out loud. Only entries whose
# mechanical de-underscoring reads badly are listed; the rest fall through to
# `_prettify`.
_OBJECT_WORDS = {
    "akita_black_bowl": "black bowl",
    "glazed_rim_porcelain_ramekin": "ramekin",
    "wooden_cabinet": "cabinet",
    "wooden_two_layer_shelf": "shelf",
    "white_yellow_mug": "yellow and white mug",
    "porcelain_mug": "white mug",
    "flat_stove": "stove",
    "chefmate_8_frypan": "frying pan",
    "cookies": "cookie box",
    "desk_caddy": "caddy",
    "short_fridge": "fridge",
    "wooden_tray": "tray",
    "new_salad_dressing": "salad dressing",
    "alphabet_soup": "alphabet soup",
    "cream_cheese": "cream cheese box",
    "tomato_sauce": "tomato sauce",
    "bbq_sauce": "bbq sauce",
    "orange_juice": "orange juice",
    "milk": "milk carton",
    "butter": "butter",
    "ketchup": "ketchup",
    "chocolate_pudding": "chocolate pudding",
    "main_table": "table",
    "living_room_table": "table",
    "kitchen_table": "table",
    "study_table": "table",
    "coffee_table": "table",
}

# Fixtures that are scenery, not something to ask the arm about.
_BORING_FIXTURES = {"table", "floor", "wall", "counter"}

# Things that cannot be picked up, so they are only ever placement targets.
_NOT_GRASPABLE = {
    "plate", "stove", "cabinet", "white cabinet", "microwave", "shelf", "basket",
    "tray", "wine rack", "fridge", "caddy", "table", "frying pan", "rack",
}
_TARGETS = ("plate", "basket", "stove", "tray", "cabinet", "caddy", "frying pan", "rack")
_OPENABLE = {"cabinet", "white cabinet", "microwave", "fridge", "shelf"}

SUITES = ("libero_spatial", "libero_object", "libero_goal", "libero_10")
_SUITE_ORDER = SUITES + ("libero_90",)


def _prettify(asset: str) -> str:
    base = re.sub(r"_\d+$", "", asset)
    return _OBJECT_WORDS.get(base, base.replace("_", " "))


def _sexpr_block(text: str, header: str) -> str:
    """Body of ``(:header ...)``, matched by paren depth rather than by regex."""
    start = text.find(f"(:{header}")
    if start < 0:
        return ""
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return text[start + len(header) + 2:i]
    return ""


def _declared_types(block: str) -> list[str]:
    """``bowl_1 bowl_2 - akita_black_bowl`` lines -> declared types, in order."""
    types: list[str] = []
    for line in block.splitlines():
        if "-" not in line:
            continue
        _, _, rhs = line.partition("-")
        asset = rhs.strip()
        if asset and asset not in types:
            types.append(asset)
    return types


def _language(text: str) -> str:
    return " ".join(_sexpr_block(text, "language").split())


@dataclass(frozen=True)
class Scene:
    scene_id: str
    label: str
    source: str              # "sandbox" | "libero"
    bddl: str
    suite: str = ""
    task_index: int = -1
    trained_instruction: str = ""
    objects: tuple[str, ...] = ()
    fixtures: tuple[str, ...] = ()

    @property
    def nouns(self) -> tuple[str, ...]:
        return self.objects + self.fixtures

    def to_json(self) -> dict[str, object]:
        return {
            "scene_id": self.scene_id,
            "label": self.label,
            "source": self.source,
            "suite": self.suite,
            "task_index": self.task_index,
            "trained_instruction": self.trained_instruction,
            "objects": list(self.objects),
            "fixtures": list(self.fixtures),
            "suggestions": suggestions(self),
            "ab_pair": ab_pair(self),
        }


@dataclass
class Catalog:
    scenes: dict[str, Scene] = field(default_factory=dict)

    def get(self, scene_id: str) -> Scene | None:
        return self.scenes.get(scene_id)

    def ordered(self) -> list[Scene]:
        def key(scene: Scene) -> tuple[int, int, str]:
            rank = _SUITE_ORDER.index(scene.suite) if scene.suite in _SUITE_ORDER else 99
            return (0 if scene.source == "sandbox" else 1, rank, scene.label)
        return sorted(self.scenes.values(), key=key)

    def to_json(self) -> list[dict[str, object]]:
        return [scene.to_json() for scene in self.ordered()]


def _scene_from_bddl(bddl: Path, *, scene_id: str, label: str, source: str,
                     suite: str = "", task_index: int = -1) -> Scene:
    text = bddl.read_text()
    objects = tuple(dict.fromkeys(
        _prettify(a) for a in _declared_types(_sexpr_block(text, "objects"))))
    fixtures = tuple(
        f for f in dict.fromkeys(
            _prettify(a) for a in _declared_types(_sexpr_block(text, "fixtures")))
        if f not in _BORING_FIXTURES and f not in objects
    )
    return Scene(
        scene_id=scene_id, label=label, source=source, bddl=str(bddl),
        suite=suite, task_index=task_index, trained_instruction=_language(text),
        objects=objects, fixtures=fixtures,
    )


def sandbox_scenes(scenes_dir: Path) -> list[Scene]:
    return [
        _scene_from_bddl(bddl, scene_id=f"sandbox:{bddl.stem}",
                         label=f"sandbox / {bddl.stem.replace('_', ' ')}", source="sandbox")
        for bddl in sorted(scenes_dir.glob("*.bddl"))
    ]


def libero_scenes(suites: tuple[str, ...] = SUITES) -> list[Scene]:
    """Every installed LIBERO world. Requires LIBERO importable and torch stubbed."""
    from libero.libero import benchmark, get_libero_path

    root = Path(get_libero_path("bddl_files"))
    registry = benchmark.get_benchmark_dict()
    out: list[Scene] = []
    for suite in suites:
        factory = registry.get(suite)
        if factory is None:
            continue
        bm = factory()
        for index in range(bm.n_tasks):
            task = bm.get_task(index)
            bddl = root / task.problem_folder / task.bddl_file
            if not bddl.exists():
                continue
            out.append(_scene_from_bddl(
                bddl, scene_id=f"{suite}:{index}",
                label=f"{suite.removeprefix('libero_')} #{index} · "
                      f"{Path(task.bddl_file).stem.replace('_', ' ')}",
                source="libero", suite=suite, task_index=index))
    return out


def build_catalog(scenes_dir: Path, *, include_libero: bool = True,
                  suites: tuple[str, ...] = SUITES) -> Catalog:
    catalog = Catalog()
    for scene in sandbox_scenes(scenes_dir):
        catalog.scenes[scene.scene_id] = scene
    if include_libero:
        for scene in libero_scenes(suites):
            catalog.scenes[scene.scene_id] = scene
    return catalog


def _graspable(scene: Scene) -> list[str]:
    return [n for n in scene.objects if n not in _NOT_GRASPABLE]


def _targets(scene: Scene) -> list[str]:
    return [n for n in scene.nouns if n in _TARGETS]


def suggestions(scene: Scene) -> list[dict[str, str]]:
    """VLA prompt chips grounded in what this scene actually contains.

    ``kind`` is the honesty knob the UI renders:
      ``trained``  the instruction this world ships with -- the phrasing the
                   policy was demonstrated on;
      ``unseen``   this world's objects, a phrasing or goal it was never shown.
                   In-distribution *skill*, out-of-distribution *sentence*: the
                   interesting case, and what "flexible" has to mean here.
    """
    chips: list[dict[str, str]] = []
    # Only a LIBERO world ships a *demonstrated* instruction. A local sandbox's
    # `:language` line is a description this repository wrote, so labelling it
    # "trained" would invent provenance the policy never had.
    if scene.trained_instruction and scene.source == "libero":
        chips.append({"text": scene.trained_instruction.lower(), "kind": "trained"})

    graspable, targets = _graspable(scene), _targets(scene)
    for noun in graspable[:3]:
        chips.append({"text": f"pick up the {noun}", "kind": "unseen"})
    if graspable and targets:
        chips.append({"text": f"put the {graspable[0]} on the {targets[0]}", "kind": "unseen"})
    if len(graspable) >= 2:
        chips.append({"text": f"move the {graspable[0]} next to the {graspable[1]}",
                      "kind": "unseen"})
    for fixture in scene.fixtures:
        if fixture in _OPENABLE:
            chips.append({"text": f"open the {fixture}", "kind": "unseen"})
        elif fixture == "stove":
            chips.append({"text": "turn on the stove", "kind": "unseen"})

    seen: set[str] = set()
    unique: list[dict[str, str]] = []
    for chip in chips:
        if chip["text"] not in seen:
            seen.add(chip["text"])
            unique.append(chip)
    return unique[:8]


def ab_pair(scene: Scene) -> list[str]:
    """Two prompts that differ only in the object referred to.

    This is what the A/B probe compares from one frozen observation: identical
    pixels, identical proprioception, one noun changed. Any difference in the
    action chunk is attributable to language and nothing else.
    """
    graspable = _graspable(scene)
    if len(graspable) >= 2:
        return [f"pick up the {graspable[0]}", f"pick up the {graspable[1]}"]
    others = [n for n in scene.nouns if graspable and n != graspable[0]]
    if graspable and others:
        return [f"pick up the {graspable[0]}", f"pick up the {others[0]}"]
    return []


if __name__ == "__main__":
    import os
    import sys

    os.environ.setdefault("LIBERO_CONFIG_PATH", "/home/ubuntu/libero/config")
    if Path("/home/ubuntu/libero/LIBERO").exists():
        sys.path.insert(0, "/home/ubuntu/libero/LIBERO")
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from libero_closed_loop import install_torch_stub

    install_torch_stub()
    catalog = build_catalog(Path(__file__).resolve().parent / "scenes")
    print(f"{len(catalog.scenes)} scenes\n")
    for scene in catalog.ordered():
        print(f"{scene.scene_id:20s} {scene.label}")
        print(f"  objects: {', '.join(scene.nouns) or '(none)'}")
        print(f"  A/B    : {' | '.join(ab_pair(scene)) or '(none)'}")
        for chip in suggestions(scene):
            print(f"  [{chip['kind']:7s}] {chip['text']}")
