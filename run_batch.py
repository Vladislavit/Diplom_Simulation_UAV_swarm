# run_batch.py — автоматичний батч: усі стратегії × сиди, збір метрик.
#
# Кожен прогін — окремий headless-процес main.py, що сам зберігає CSV+PNG
# у results/{strategy}_{timestamp}/ і завершується при кінці місії.

import subprocess
import sys
import time

STRATEGIES = ['greedy', 'swarm_only', 'auction',
              'hybrid', 'leader_follower']
SEEDS = [42, 7, 123, 256, 999]          # 5 прогонів на стратегію
RUNS_PER_STRATEGY = len(SEEDS)


def main():
    total = len(SEEDS) * len(STRATEGIES)
    done = 0
    failed = []
    start = time.time()

    for seed in SEEDS:
        for strategy in STRATEGIES:
            done += 1
            print(f"[{done}/{total}] Running {strategy} seed={seed}...",
                  flush=True)
            result = subprocess.run([
                sys.executable, 'main.py',
                '--headless',
                f'--strategy={strategy}',
                f'--seed={seed}',
            ])
            if result.returncode != 0:
                failed.append((strategy, seed, result.returncode))
                print(f"   ! завершився з кодом {result.returncode}",
                      flush=True)
            time.sleep(0.5)

    elapsed = time.time() - start
    print(f"\nВсі запуски завершено за {elapsed:.0f}с "
          f"({total} прогонів). Результати в results/")
    if failed:
        print(f"Невдалі прогони ({len(failed)}): {failed}")


if __name__ == '__main__':
    main()
