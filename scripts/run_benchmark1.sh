for i in {1..4}
do
    python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 2000 --max_depth 30 --algo bai_perturb --debug_mode false --box_number 6 --seed $i &
    # python3 -m multi_armed_bandit.run_grid_search
done
python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 2000 --max_depth 30 --algo bai_perturb --debug_mode false --box_number 6 --seed 5
