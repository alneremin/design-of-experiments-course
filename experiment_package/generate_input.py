#!/usr/bin/env python3
import numpy as np
import itertools
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--values", type=float, nargs="+", default=[0.1, 0.5, 1.0, 2.0, 3.0])
    parser.add_argument("--repeats", type=int, default=10, help="Количество повторов на комбинацию")
    parser.add_argument("--output", type=str, default="input.npy")
    args = parser.parse_args()

    vals = args.values
    repeats = args.repeats

    trials = []
    global_idx = 0
    for path_align, goal_align in itertools.product(vals, repeat=2):
        for rep in range(repeats):
            # Сохраняем: [path_align, goal_align, global_index]
            trials.append([float(path_align), float(goal_align), float(global_idx)])
            global_idx += 1

    exp_plan = np.array(trials, dtype=np.float64)
    np.save(args.output, exp_plan)

    print(f"Создано {len(trials)} экспериментов ({len(vals)}×{len(vals)} комбинаций × {repeats} повторов)")
    print(f"Пример первых 5 строк:")
    for i, row in enumerate(exp_plan[:5]):
        print(f"  {i}: {row}")

if __name__ == "__main__":
    main()