import csv
import math
from pathlib import Path


def load(path):
    points = []
    with Path(path).open(newline="") as handle:
        for row in csv.reader(handle):
            if not row or row[0].strip().startswith("#"):
                continue
            points.append((float(row[0]), float(row[1])))
    return points


def audit(path):
    points = load(path)
    length = 0.0
    max_step = 0.0
    reversals = 0
    last_heading = None
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        step = math.hypot(bx - ax, by - ay)
        length += step
        max_step = max(max_step, step)
        heading = math.atan2(by - ay, bx - ax)
        if last_heading is not None:
            delta = abs(math.atan2(math.sin(heading - last_heading), math.cos(heading - last_heading)))
            if delta > math.radians(120):
                reversals += 1
        last_heading = heading
    print(f"{path}: points={len(points)} length_m={length:.2f} max_step_m={max_step:.2f} sharp_reversals={reversals}")


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+")
    args = parser.parse_args()
    for path in args.paths:
        audit(path)


if __name__ == "__main__":
    main()
