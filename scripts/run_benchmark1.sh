# for i in {1..4}
# do
#     python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo bai_perturb --debug_mode false --box_number 6 --seed $i &
# done
# python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo bai_perturb --debug_mode false --box_number 6 --seed 5

# for i in {1..4}
# do
#     python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo uct --debug_mode false --box_number 6 --seed $i &
# done
# python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo uct --debug_mode false --box_number 6 --seed 5

# for i in {1..4}
# do
#     python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo random --debug_mode false --box_number 6 --seed $i &
# done
# python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo random --debug_mode false --box_number 6 --seed 5

# for i in {1..4}
# do
#     python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo bai_ucb --debug_mode false --box_number 6 --seed $i &
# done
# python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo bai_ucb --debug_mode false --box_number 6 --seed 5

for i in {6..9}
do
    python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo bai_perturb --debug_mode false --box_number 6 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo bai_perturb --debug_mode false --box_number 6 --seed 10

for i in {6..9}
do
    python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo uct --debug_mode false --box_number 6 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo uct --debug_mode false --box_number 6 --seed 10

for i in {6..9}
do
    python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo random --debug_mode false --box_number 6 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo random --debug_mode false --box_number 6 --seed 10

for i in {6..9}
do
    python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo bai_ucb --debug_mode false --box_number 6 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark1_test.py --budgets 100 --max_depth 16 --algo bai_ucb --debug_mode false --box_number 6 --seed 10
