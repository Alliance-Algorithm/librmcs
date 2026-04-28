#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

parse_version() {
    local version_output="$1"
    local parsed_version=""

    parsed_version=$(printf '%s' "$version_output" | grep -Eo 'v?[0-9]+([.][0-9]+)*([-.][0-9A-Za-z]+)*' | head -n 1 || true)

    if [ -n "$parsed_version" ] && printf '%s' "$parsed_version" | grep -q '[0-9]'; then
        parsed_version=${parsed_version#v}
        printf '%s' "$parsed_version"
        return 0
    fi

    return 1
}

probe_tool() {
    tool_name="$1"

    if ! command -v "$tool_name" >/dev/null 2>&1; then
        printf '%s\n' 'absent'
        return 0
    fi

    version_output=""
    if version_output=$($tool_name --version 2>/dev/null); then
        if version=$(parse_version "$version_output"); then
            printf '%s\n' "$version"
        else
            printf '%s\n' 'unparseable'
        fi
    else
        printf '%s\n' 'unparseable'
    fi
}

generated_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
manifest_dir="$REPO_ROOT/.devcontainer/.generated"
manifest_path="$manifest_dir/host-tools.manifest"
tmp_manifest_path="$manifest_path.tmp"

mkdir -p "$manifest_dir"

codex_version="$(probe_tool codex)"
claude_version="$(probe_tool claude)"
opencode_version="$(probe_tool opencode)"
lark_cli_version="$(probe_tool lark-cli)"

{
    printf '%s\n' 'schema=1'
    printf 'generated_at=%s\n' "$generated_at"
    printf 'codex=%s\n' "$codex_version"
    printf 'claude=%s\n' "$claude_version"
    printf 'opencode=%s\n' "$opencode_version"
    printf 'lark-cli=%s\n' "$lark_cli_version"
} > "$tmp_manifest_path"

mv "$tmp_manifest_path" "$manifest_path"

printf 'Host tool probe summary:\n'
printf '  codex=%s\n' "$codex_version"
printf '  claude=%s\n' "$claude_version"
printf '  opencode=%s\n' "$opencode_version"
printf '  lark-cli=%s\n' "$lark_cli_version"
printf '  manifest=%s\n' "$manifest_path"
