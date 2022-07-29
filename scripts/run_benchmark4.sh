for i in {1..4}
do
    python3 ../examples/doosan/mcts/benchmark4_test.py --budgets 1000 --max_depth 20 --algo bai_perturb --debug_mode False --seed $i &
done
python3 ../examples/doosan/mcts/benchmark4_test.py --budgets 1000 --max_depth 20 --algo bai_perturb --debug_mode False --seed 5
