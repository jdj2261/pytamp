for i in 1 2 3 4
do
    python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo bai_perturb --debug_mode False --seed $i &
done
python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo bai_perturb --debug_mode False --seed 5

for i in 1 2 3 4
do
    python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo uct --debug_mode False --seed $i &
done
python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo uct --debug_mode False --seed 5

for i in 1 2 3 4
do
    python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo random --debug_mode False --seed $i &
done
python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo random --debug_mode False --seed 5

for i in 1 2 3 4
do
    python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo bai_ucb --debug_mode False --seed $i &
done
python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo bai_ucb --debug_mode False --seed 5

# for i in 6 7 8 9
# do
#     python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo bai_perturb --debug_mode False --seed $i &
# done
# python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo bai_perturb --debug_mode False --seed 10

# for i in 6 7 8 9
# do
#     python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo uct --debug_mode False --seed $i &
# done
# python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo uct --debug_mode False --seed 10

# for i in 6 7 8 9
# do
#     python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo random --debug_mode False --seed $i &
# done
# python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo random --debug_mode False --seed 10

# for i in 6 7 8 9
# do
#     python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo bai_ucb --debug_mode False --seed $i &
# done
# python3 -m examples.doosan.mcts.benchmark2_test --budgets 100 --max_depth 16 --algo bai_ucb --debug_mode False --seed 10
