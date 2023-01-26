# for i in {1..4}
# do
#     python3 -m examples.doosan.mcts.benchmark3_test.py --budgets 100 --max_depth 14 --algo bai_perturb --debug_mode False --seed $i &
# done
# python3 -m examples.doosan.mcts.benchmark3_test.py --budgets 100 --max_depth 14 --algo bai_perturb --debug_mode False --seed 5

# for i in {1..4}
# do
#     python3 -m examples.doosan.mcts.benchmark3_test.py --budgets 100 --max_depth 14 --algo uct --debug_mode False --seed $i &
# done
# python3 -m examples.doosan.mcts.benchmark3_test.py --budgets 100 --max_depth 14 --algo uct --debug_mode False --seed 5

for i in {1..4}
do
    python3 -m examples.doosan.mcts.benchmark3_test.py --budgets 100 --max_depth 14 --algo random --debug_mode False --seed $i &
done
python3 -m examples.doosan.mcts.benchmark3_test.py --budgets 100 --max_depth 14 --algo random --debug_mode False --seed 5

for i in {1..4}
do
    python3 -m examples.doosan.mcts.benchmark3_test.py --budgets 100 --max_depth 14 --algo bai_ucb --debug_mode False --seed $i &
done
python3 -m examples.doosan.mcts.benchmark3_test.py --budgets 100 --max_depth 14 --algo bai_ucb --debug_mode False --seed 5
