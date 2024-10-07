#!/usr/bin/env bash

set -x -e -u -o pipefail

cd "$(dirname "$0")/../../"

source ci/includes/os.sh

MAYBEARG='-mode=check'
if [ $# -eq 1 ]; then
  if [ "$1" = "-fix" ]; then
    echo -e "\033[0;34mAttempting to fix linting errors automatically as '-fix' is specified.\033[0m"
    MAYBEARG=''
  fi
fi

# Ensure Bazel is installed.
bazel version

if bazel run buildifier -- ${MAYBEARG} -v $(find "$(pwd)/" \( -name BUILD -o -name WORKSPACE \) -type f); then
  echo -e "\033[0;32mAll BUILD and WORKSPACE files passed buildifier linting check.\033[0m"
else
  echo -e "\033[0;31mThe above listed BUILD and WORKSPACE file(s) didn't pass the buildifier linting check!\033[0m"
  echo -e "\033[0;34mYou can run 'ci/linting/buildifier.sh -fix' to fix them automatically.\033[0m"
  exit 1
fi

