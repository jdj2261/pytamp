for i in {1..4}
do
    python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 500 --max_depth 20 --algo bai_perturb --debug_mode false --box_number 6 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 500 --max_depth 20 --algo bai_perturb --debug_mode false --box_number 6 --seed 5
