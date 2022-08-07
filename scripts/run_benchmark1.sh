for i in {1..2}
do
    python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 5 --max_depth 16 --algo bai_perturb --debug_mode false --box_number 5 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 5 --max_depth 16 --algo bai_perturb --debug_mode false --box_number 5 --seed 3

for i in {1..2}
do
    python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 5 --max_depth 16 --algo uct --debug_mode false --box_number 5 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 5 --max_depth 16 --algo uct --debug_mode false --box_number 5 --seed 3

for i in {1..2}
do
    python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 5 --max_depth 16 --algo random --debug_mode false --box_number 5 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 5 --max_depth 16 --algo random --debug_mode false --box_number 5 --seed 3
