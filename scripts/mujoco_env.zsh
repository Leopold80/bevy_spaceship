#!/usr/bin/env zsh

_APOLLO_SCRIPT_DIR="${${(%):-%N}:A:h}"
_APOLLO_REPO_ROOT="${_APOLLO_SCRIPT_DIR:h}"

export MUJOCO_HOME="${_APOLLO_REPO_ROOT}/.local/mujoco/3.9.0/macos"
export MUJOCO_DYNAMIC_LINK_DIR="${MUJOCO_HOME}"

# 激活 Conda 后再 source 本脚本时，同时保留其 libpython 搜索目录；这使
# `cargo test --workspace --all-targets` 能运行 PyO3 cdylib 的空测试壳。
if [[ -n "${CONDA_PREFIX:-}" && -d "${CONDA_PREFIX}/lib" ]]; then
    export DYLD_LIBRARY_PATH="${MUJOCO_HOME}:${CONDA_PREFIX}/lib${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"
else
    export DYLD_LIBRARY_PATH="${MUJOCO_HOME}${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"
fi

unset _APOLLO_SCRIPT_DIR _APOLLO_REPO_ROOT
