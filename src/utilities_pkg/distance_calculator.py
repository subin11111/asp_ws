#!/usr/bin/env python3
"""
marker_error.py ── 마커 위치 오차 계산 스크립트
Usage:  python marker_error.py ground_truth.csv estimate.csv
        (각 CSV는 헤더 없고 'id,x,y,z' 형식)
"""
import sys
from pathlib import Path
import pandas as pd
import numpy as np


def load_csv(path: str) -> pd.DataFrame:
    """CSV → DataFrame(id,x,y,z). 헤더가 없다면 수동 지정."""
    cols = ["id", "x", "y", "z"]
    return pd.read_csv(path, header=None, names=cols, dtype={"id": int})


def main(gt_path: str, est_path: str) -> None:
    gt = load_csv(gt_path)
    est = load_csv(est_path)

    # 0~9 마커만 필터링
    gt = gt[gt["id"].between(0, 9)]
    est = est[est["id"].between(0, 9)]

    # id 기준 inner-join
    merged = pd.merge(gt, est, on="id", suffixes=("_gt", "_est"))
    if merged.empty:
        print("⚠️  ID 0~9 중 매칭되는 마커가 없습니다.")
        return

    # 오차 벡터 및 크기
    merged["dx"] = merged["x_est"] - merged["x_gt"]
    merged["dy"] = merged["y_est"] - merged["y_gt"]
    merged["dz"] = merged["z_est"] - merged["z_gt"]
    merged["error"] = np.linalg.norm(merged[["dx", "dy", "dz"]].to_numpy(), axis=1)

    # 결과 출력
    print("─ 마커 위치 오차 (단위: m) ─────────────────────────────")
    print(
        merged[["id", "dx", "dy", "dz", "error"]]
        .sort_values("id")
        .to_string(index=False, float_format="%.4f")
    )
    rmse = np.sqrt((merged["error"] ** 2).mean())
    print(f"\nRMSE (0–9 평균 제곱근 오차): {rmse:.4f} m")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        prog = Path(sys.argv[0]).name
        print(f"Usage: {prog} ground_truth.csv estimate.csv")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
