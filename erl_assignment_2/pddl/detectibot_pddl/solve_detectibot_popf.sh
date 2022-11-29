#! /bin/bash

echo "===== solving with popf... ====="
timeout 10 ../popf db_domain.pddl db_problem.pddl > solution.txt
echo "===== ...done ====="
