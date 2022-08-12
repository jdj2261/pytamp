for i in {1..4}
do
    python3 -m examples.doosan.mcts.benchmark4_test.py --budgets 100 --max_depth 32 --algo bai_perturb --debug_mode False --disk_number 3 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark4_test.py --budgets 100 --max_depth 32 --algo bai_perturb --debug_mode False --disk_number 3 --seed 5

for i in {1..4}
do
    python3 -m examples.doosan.mcts.benchmark4_test.py --budgets 100 --max_depth 32 --algo uct --debug_mode False --disk_number 3 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark4_test.py --budgets 100 --max_depth 32 --algo uct --debug_mode False --disk_number 3 --seed 5

for i in {1..4}
do
    python3 -m examples.doosan.mcts.benchmark4_test.py --budgets 100 --max_depth 32 --algo random --debug_mode False --disk_number 3 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark4_test.py --budgets 100 --max_depth 32 --algo random --debug_mode False --disk_number 3 --seed 5

for i in {1..4}
do
    python3 -m examples.doosan.mcts.benchmark4_test.py --budgets 100 --max_depth 32 --algo bai_ucb --debug_mode False --disk_number 3 --seed $i &
done
python3 -m examples.doosan.mcts.benchmark4_test.py --budgets 100 --max_depth 32 --algo bai_ucb --debug_mode False --disk_number 3 --seed 5
