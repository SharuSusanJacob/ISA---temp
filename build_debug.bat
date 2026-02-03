@echo off
gcc -Wall -Wextra -O0 -g -Isrc -Iinclude fswtest.c src/major.c src/minor.c src/math_utils.c -o fswtest.exe -lm > build_output.txt 2>&1
echo Build finished. Output captured in build_output.txt
