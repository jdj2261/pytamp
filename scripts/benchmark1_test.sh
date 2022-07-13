for i in {1..4}
do
    python3 example/bench_1_test.py --budgets 1 --max_depth 1 --algo bai_ucb --seed $i &
done
python3 bench_1_test.py --budgets 1 --max_depth 1 --algo bai_ucb --seed 5
