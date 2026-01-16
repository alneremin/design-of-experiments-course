#!/usr/bin/env python3
import numpy as np
import sys

def main():
    path = "input.npy"
    if len(sys.argv) > 1:
        path = sys.argv[1]

    try:
        data = np.load(path, allow_pickle=True)
        print(f"Загружен файл: {path}")
        print(f"Форма массива: {data.shape}")
        print("\nПервые 10 строк:")
        for i, row in enumerate(data[:10]):
            print(f"  {i}: {row}")
        if data.shape[0] > 10:
            print("  ...")
            print(f"  Всего экспериментов: {data.shape[0]}")
    except FileNotFoundError:
        print(f"Ошибка: файл {path} не найден.")
    except Exception as e:
        print(f"Ошибка при загрузке: {e}")

if __name__ == "__main__":
    main()