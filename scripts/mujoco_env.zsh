#!/usr/bin/env zsh

export MUJOCO_HOME="${PWD}/.local/mujoco/3.9.0/macos"
export MUJOCO_DYNAMIC_LINK_DIR="${MUJOCO_HOME}"
export DYLD_LIBRARY_PATH="${MUJOCO_HOME}${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"
