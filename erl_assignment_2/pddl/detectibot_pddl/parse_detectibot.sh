#!/bin/bash

echo "===== running parser... ====="
../parser ./detectibot_pddl/db_domain.pddl ./detectibot_pddl/db_problem.pddl > parselog.log
echo "===== ...done ====="
