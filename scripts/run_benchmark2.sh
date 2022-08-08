for i in {1..4}
do
    python3 examples.doosan.mcts.benchmark2_test.py --budgets 100 --max_depth 20 --algo bai_perturb --debug_mode False --seed $i &
done
python3 examples.doosan.mcts.benchmark2_test.py --budgets 100 --max_depth 20 --algo bai_perturb --debug_mode False --seed 5

for i in {1..4}
do
    python3 examples.doosan.mcts.benchmark2_test.py --budgets 100 --max_depth 20 --algo uct --debug_mode False --seed $i &
done
python3 examples.doosan.mcts.benchmark2_test.py --budgets 100 --max_depth 20 --algo uct --debug_mode False --seed 5

for i in {1..4}
do
    python3 examples.doosan.mcts.benchmark2_test.py --budgets 100 --max_depth 20 --algo random --debug_mode False --seed $i &
done
python3 examples.doosan.mcts.benchmark2_test.py --budgets 100 --max_depth 20 --algo random --debug_mode False --seed 5
