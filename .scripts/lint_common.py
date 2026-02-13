"""Shared utilities for lint scripts."""

import yaml
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple


SOURCE_EXTENSIONS = {".c", ".cpp", ".h", ".hpp"}


@dataclass(frozen=True)
class LintTarget:
    name: str
    compile_database: Path
    clangd: Optional[Path]
    folders: List[Path]


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


def load_targets(root: Path) -> Tuple[List[str], Dict[str, LintTarget]]:
    config_path = root / ".scripts" / "lint-targets.yml"
    data = yaml.safe_load(config_path.read_text())

    exclude_dirs: List[str] = data.pop("exclude", [])
    targets: Dict[str, LintTarget] = {}
    for name, section in data.items():
        compile_database = root / section["compile_database"]
        clangd_rel = section.get("clangd")
        clangd = root / clangd_rel if clangd_rel else None
        folders = [root / f for f in section["folders"]]
        targets[name] = LintTarget(
            name=name,
            compile_database=compile_database,
            clangd=clangd,
            folders=folders,
        )
    return exclude_dirs, targets
