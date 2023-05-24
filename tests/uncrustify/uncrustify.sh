#!/bin/bash
# This script is intended to lint the code, it will replace the files in place
# Usage: ./tests/uncrustify/uncrustify.sh
sh -c 'find Drivers Middlewares Projects -type f -name "*.c" -o -name "*.cpp" -o -name "*.h" | uncrustify -c tests/uncrustify/converge.cfg -F - --replace --no-backup'

