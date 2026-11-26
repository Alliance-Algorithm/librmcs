"""Shared utilities for lint scripts."""

import json
import yaml
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Set, Tuple


SOURCE_EXTENSIONS = {".c", ".cpp", ".h", ".hpp"}


@dataclass(frozen=True)
class LintTarget:
    name: str
    project_dir: Path
    folders: List[Path]
    build_dirs: Tuple[str, ...] = ("build",)

    @property
    def compile_databases(self) -> List[Path]:
        return [
            self.project_dir / build_dir / "compile_commands.json"
            for build_dir in self.build_dirs
        ]


def repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def should_exclude(path: Path, root: Path, exclude_dirs: List[str]) -> bool:
    try:
        rel_parts = path.resolve().relative_to(root).parts
    except ValueError:
        return True
    for d in exclude_dirs:
        if d in rel_parts:
            return True
    return False


def collect_source_files(
    dirs: Sequence[Path], root: Path, exclude_dirs: List[str]
) -> List[Path]:
    files: Set[Path] = set()
    for d in dirs:
        if not d.is_dir():
            continue
        for p in d.rglob("*"):
            if not p.is_file():
                continue
            if p.suffix not in SOURCE_EXTENSIONS:
                continue
            if should_exclude(p, root=root, exclude_dirs=exclude_dirs):
                continue
            files.add(p.resolve())
    return sorted(files)


def load_compile_database(database: Path) -> Dict[Path, Path]:
    """Map every translation unit to the build directory passed to clang-tidy."""

    entries: Dict[Path, Path] = {}
    for entry in json.loads(database.read_text()):
        entries.setdefault(Path(entry["file"]).resolve(), database.parent)
    return entries


def load_targets(root: Path) -> Tuple[List[str], Dict[str, LintTarget]]:
    config_path = root / ".scripts" / "lint-targets.yml"
    data = yaml.safe_load(config_path.read_text())

    exclude_dirs: List[str] = data.pop("exclude", [])
    targets: Dict[str, LintTarget] = {}
    for name, section in data.items():
        project_dir = root / Path(section["cmake"]).parent
        folders = [root / f for f in section["folders"]]
        build_dirs = tuple(section.get("build_dirs", ["build"]))
        targets[name] = LintTarget(
            name=name,
            project_dir=project_dir,
            folders=folders,
            build_dirs=build_dirs,
        )
    return exclude_dirs, targets
