find . \( -path "./include/*" -o -path "./src/*" -o -path "./test/*" -o -path "./demo/*" \) -a \( -path "*.h" -o -path "*.cpp" \) -print | xargs clang-format -i
