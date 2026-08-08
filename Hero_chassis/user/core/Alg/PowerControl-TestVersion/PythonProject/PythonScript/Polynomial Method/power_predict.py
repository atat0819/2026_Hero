# -*- coding: utf-8 -*-
"""
power_predict.py — 电机输入功率 6 系数拟合脚本(功率计实测数据版)
===============================================================

公式: P = K0 + K1*I + K2*w + K3*I*w + K4*I^2 + K5*w^2
       I: 电流 (A)   w: 转子转速 (rad/s)   P: 输入功率 (W)

支持两种数据模式 (NUM_MOTORS 配置):

  1) 单电机模式 (NUM_MOTORS=1):
     通道: [P_in, w, I, target, P_est, ...]   (6 列, 原 vofa_send)
     拟合单电机输入功率。

  2) 整车模式 (NUM_MOTORS=4, 默认):
     通道: [P_total, w1, I1, w2, I2, w3, I3, w4, I4]   (9 列, 需 vofa_send9)
     功率计测整车总功率, 4 个电机共享同一组系数:
       P_total = K0*4 + K1*(ΣI) + K2*(Σw) + K3*(Σ I·w) + K4*(Σ I²) + K5*(Σ w²)
     这与固件 can_send_task.cpp 里 post_power 的求和结构完全一致,
     拟合出的 6 个系数可直接替换 poly_coeffs。

为什么不用仓库里的 Power.py:
    Power.py 直接把 CSV 丢进最小二乘, 不做任何数据质量检查。
    若功率计读数不可靠 (接线/量程错误), 或工况覆盖不全 (比如空转斜坡
    缺少"高转速+大电流"区域), 拟合出的系数必然错误, 实车预测自然对不上。

本脚本的改进:
    1. 数据质量诊断: 打印各通道统计量, 检测功率计量级是否合理
    2. 工况覆盖检查: (w, I) 联合分布表, 一眼看出哪个区域没有数据
    3. 异常点剔除:   按物理极限剔除不可能的点
    4. 可选滑窗滤波: 处理 6020 直驱电机那种噪声数据
    5. holdout 验证:  末尾 20% 数据不参与拟合, 单独评估泛化能力
    6. 固件 P_est 对比: 日志里带 P_est 通道时自动对比新旧模型
    7. 输出 C 宏定义, 可直接贴入 can_send_task.cpp

使用方法:
    python power_predict.py                    # 默认配置
    python power_predict.py E:\\data.csv       # 指定 CSV 路径
    修改下方"配置区"可调整通道映射 / 清洗参数。

依赖: numpy pandas matplotlib   (pip install numpy pandas matplotlib)
"""

import os
import sys
try:
    sys.stdout.reconfigure(encoding='utf-8')  # 避免 Windows GBK 控制台中文乱码/报错
except AttributeError:
    pass
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # 不弹窗, 直接存图 (需要看图的去掉这行)
import matplotlib.pyplot as plt

# ============================================================
#                      配置区
# ============================================================

CSV_PATH = r"E:\power_800.csv"   # 拟合数据 (可在命令行传入覆盖)

NUM_MOTORS = 4     # 1 = 单电机; 4 = 整车 4 电机 (通道: P, w1, I1, w2, I2, ...)
CH_PIN = 0         # 功率计通道 (单电机: 单电机输入功率; 整车: 整车总功率)
CH_TGT = None      # 目标转速列 (仅画图参考, 单电机时可为 3, 整车时无, 设 None)
CH_EST = None      # 固件 P_est 列 (可选对比, 没有就设 None; 单电机模式通常为 4)

# --- 数据清洗 ---
FILTER_WINDOW  = 0      # 滑窗均值窗口 (点数)。0 = 关闭。6020 类噪声数据建议 20~50
DROP_HARD_LIM  = True   # 剔除物理上不可能的点 (电流/转速/功率超限)
I_LIMIT        = 25.0   # 电流极限 (A), 3508 峰值 20A
W_LIMIT        = 950.0  # 转速极限 (rad/s), 3508 转子约 930 rad/s
P_LIMIT        = 2000.0 # 功率极限 (W): 整车模式给大些 (4 电机), 单电机给 ~600

# --- 验证 ---
VAL_FRACTION   = 0.2    # 末尾 20% 作为 holdout 验证集 (不参与拟合)

# --- 物理参考 (仅打印提示用, 不参与计算) ---
KT_REF     = 0.0105     # 3508 转子转矩常数 (Nm/A), 用于参考量级
R_PH_REF   = 0.28       # 3508 相电阻 (Ohm), K4 拟合值应接近 3*R_PH_REF

# ============================================================
#                     核心代码
# ============================================================


def load_data(path):
    """加载 VOFA+ 导出的 CSV, 返回 DataFrame"""
    if not os.path.exists(path):
        sys.exit(f"错误: 文件不存在 - {path}")
    df = pd.read_csv(path)
    print(f"加载文件: {path}")
    print(f"数据形状: {df.shape[0]} 行 x {df.shape[1]} 列  (模式: {'整车 %d 电机' % NUM_MOTORS if NUM_MOTORS > 1 else '单电机'})")
    need = 1 + 2 * NUM_MOTORS
    if df.shape[1] < need:
        sys.exit(f"错误: {NUM_MOTORS} 电机模式需要至少 {need} 列, 文件只有 {df.shape[1]} 列")
    return df


def extract(df):
    """按通道映射提取物理量: 返回 (P, W, I, tgt, P_est)
    P:  (n,)          功率计读数
    W:  (n, NUM_MOTORS) 各电机转速
    I:  (n, NUM_MOTORS) 各电机电流
    """
    def col(idx):
        return pd.to_numeric(df.iloc[:, idx], errors='coerce').values

    P = col(CH_PIN)
    W = np.column_stack([col(1 + 2 * i) for i in range(NUM_MOTORS)])
    I = np.column_stack([col(2 + 2 * i) for i in range(NUM_MOTORS)])
    tgt = col(CH_TGT) if CH_TGT is not None else None
    P_est = col(CH_EST) if CH_EST is not None else None
    return P, W, I, tgt, P_est


def diagnostics(P, W, I):
    """打印数据统计 + 物理合理性检查"""
    n = len(P)
    print("\n" + "=" * 60)
    print("数据统计")
    print("=" * 60)
    print(f"  功率计 P : min={np.nanmin(P):10.3f}  max={np.nanmax(P):10.3f}"
          f"  mean={np.nanmean(P):9.3f}  std={np.nanstd(P):8.3f}  (W)")
    for i in range(NUM_MOTORS):
        w, Ii = W[:, i], I[:, i]
        print(f"  电机{i+1}   w: min={np.nanmin(w):9.1f}  max={np.nanmax(w):9.1f}"
              f"  | I: min={np.nanmin(Ii):9.2f}  max={np.nanmax(Ii):9.2f}")

    # 工况覆盖检查: (|w|, |I|) 联合分布 (合并所有电机)
    print("\n" + "=" * 60)
    print("工况覆盖检查 ((|w|,|I|) 联合分布, 所有电机合并)")
    print("=" * 60)
    aw = np.concatenate([np.abs(W[:, i]) for i in range(NUM_MOTORS)])
    aI = np.concatenate([np.abs(I[:, i]) for i in range(NUM_MOTORS)])
    edges_w = [0, 100, 300, 500, 700, 950]
    edges_i = [0, 2, 5, 10, 25]
    hdr = "      " + "".join(f"I {a}-{b:>2d}A" for a, b in zip(edges_i[:-1], edges_i[1:]))
    print(hdr)
    for a, b in zip(edges_w[:-1], edges_w[1:]):
        row = f"w {a:>3d}-{b:>3d} "
        for c, d in zip(edges_i[:-1], edges_i[1:]):
            cnt = ((aw >= a) & (aw < b) & (aI >= c) & (aI < d)).sum()
            row += f"{cnt:>9d} "
        print(row)
    # 高速大电流区提示
    hi = (aw >= 700) & (aI >= 5)
    print(f"  高速(>700)+大电流(>5A) 点数: {hi.sum()}  "
          f"({'充足' if hi.sum() > 200 else '⚠ 严重不足, 模型在此区域是纯外推! 需补录该工况数据'})")

    # 物理合理性
    print("\n" + "=" * 60)
    print("物理合理性检查 (功率计量级)")
    print("=" * 60)
    Pp = P[~np.isnan(P)]
    rms_p = np.sqrt(np.mean(Pp**2))
    print(f"  功率计 RMS = {rms_p:.2f} W, 负功率(回馈)占比 {100*(Pp < -1).mean():.1f}%")
    if rms_p < 5 and NUM_MOTORS == 1:
        print(f"  ⚠ 功率计 RMS 只有 {rms_p:.2f} W, 量级可能不对! "
              f"3508 正常测试应有 10W+ 量级, 请检查功率计接线/量程后再录")
    if np.nanmax(np.abs(P)) > 1e4:
        print(f"  ⚠ 第一列(功率计)数值范围 {np.nanmin(P):.3g} ~ {np.nanmax(P):.3g} 异常偏大!")
        print(f"    可能 VOFA+ 导出带了时间戳/其他大数值列, 请检查导出设置/通道映射后再拟合!")
    # 时间戳特征: 严格单调递增的列几乎不可能是功率计
    Pn = P[~np.isnan(P)]
    if len(Pn) > 100:
        mono = np.mean(np.diff(Pn) >= 0)
        if mono > 0.99:
            print(f"  ⚠ 第一列几乎严格单调递增 (非减比例 {mono*100:.1f}%), 疑似时间戳列!")
            print(f"    可能 VOFA+ 导出时勾选了时间列(第一列是时间, 通道整体错位)。")
            print(f"    请在 VOFA+ 导出时去掉时间戳, 或调整通道映射后重新拟合!")


def clean_data(P, W, I, P_est, tgt):
    """异常点剔除 + 可选滤波"""
    n0 = len(P)
    mask = ~np.isnan(P)
    for i in range(NUM_MOTORS):
        mask &= ~(np.isnan(W[:, i]) | np.isnan(I[:, i]))
    if DROP_HARD_LIM:
        for i in range(NUM_MOTORS):
            mask &= (np.abs(I[:, i]) <= I_LIMIT) & (np.abs(W[:, i]) <= W_LIMIT)
        mask &= np.abs(P) <= P_LIMIT
    P, W, I = P[mask], W[mask], I[mask]
    if P_est is not None:
        P_est = P_est[mask]
    if tgt is not None:
        tgt = tgt[mask]
    if DROP_HARD_LIM:
        print(f"\n[清洗] 剔除超限/无效点 {n0 - len(P)} 个 (剩余 {len(P)})")

    if FILTER_WINDOW > 1:
        s = pd.Series
        P = s(P).rolling(FILTER_WINDOW, center=True, min_periods=1).mean().values
        for i in range(NUM_MOTORS):
            W[:, i] = s(W[:, i]).rolling(FILTER_WINDOW, center=True, min_periods=1).mean().values
            I[:, i] = s(I[:, i]).rolling(FILTER_WINDOW, center=True, min_periods=1).mean().values
        print(f"[清洗] 滑窗滤波 window={FILTER_WINDOW}")
    return P, W, I, P_est, tgt


def build_features(W, I):
    """整车求和结构特征矩阵:
    [N, ΣI, Σw, Σ(I·w), Σ(I²), Σ(w²)]  — 系数即 [K0, K1, K2, K3, K4, K5]
    """
    sI = I.sum(axis=1)
    sW = W.sum(axis=1)
    sIw = (I * W).sum(axis=1)
    sI2 = (I**2).sum(axis=1)
    sW2 = (W**2).sum(axis=1)
    return np.column_stack([np.full(len(I), NUM_MOTORS), sI, sW, sIw, sI2, sW2])


def calc_power(k, W, I):
    """整车模式: P_total = K0*N + K1*ΣI + K2*Σw + K3*ΣIw + K4*ΣI² + K5*Σw²"""
    sI = I.sum(axis=1)
    sW = W.sum(axis=1)
    return (k[0] * NUM_MOTORS + k[1] * sI + k[2] * sW
            + k[3] * (I * W).sum(axis=1)
            + k[4] * (I**2).sum(axis=1)
            + k[5] * (W**2).sum(axis=1))


def evaluate(P_true, P_pred):
    """R² / RMSE / MAE / p90"""
    err = P_true - P_pred
    ss_res = np.sum(err**2)
    ss_tot = np.sum((P_true - P_true.mean())**2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    return r2, np.sqrt(np.mean(err**2)), np.mean(np.abs(err)), np.quantile(np.abs(err), 0.9)


def fit_and_report(P, W, I, P_est, tgt, path):
    """主流程: 切分 -> 拟合 -> 评估 -> 输出"""
    n = len(P)
    n_val = int(n * VAL_FRACTION)
    n_train = n - n_val

    P_tr, W_tr, I_tr = P[:n_train], W[:n_train], I[:n_train]
    P_va, W_va, I_va = P[n_train:], W[n_train:], I[n_train:]

    # ---- 拟合 ----
    X_tr = build_features(W_tr, I_tr)
    k, *_ = np.linalg.lstsq(X_tr, P_tr, rcond=None)

    # ---- 评估 ----
    P_pred_tr = calc_power(k, W_tr, I_tr)
    P_pred_va = calc_power(k, W_va, I_va)
    r2_tr, rmse_tr, mae_tr, p90_tr = evaluate(P_tr, P_pred_tr)
    r2_va, rmse_va, mae_va, p90_va = evaluate(P_va, P_pred_va)

    print("\n" + "=" * 60)
    print(f"拟合结果 (训练集 {n_train} 点 / 验证集 {n_val} 点)  模式: "
          f"{'整车' if NUM_MOTORS > 1 else '单电机'}")
    print("=" * 60)
    print(f"  {'':5s}  {'R2':>8s} {'RMSE(W)':>9s} {'MAE(W)':>8s} {'P90(W)':>8s}")
    print(f"  训练  {r2_tr:8.4f} {rmse_tr:9.3f} {mae_tr:8.3f} {p90_tr:8.3f}")
    print(f"  验证  {r2_va:8.4f} {rmse_va:9.3f} {mae_va:8.3f} {p90_va:8.3f}   <- 泛化能力看这行")

    print("\n" + "=" * 60)
    print("拟合系数 (每个电机共享同一组)")
    print("=" * 60)
    names = ['K0(常数)', 'K1(I)', 'K2(w)', 'K3(I*w)', 'K4(I^2)', 'K5(w^2)']
    for nm, v in zip(names, k):
        print(f"  {nm:10s} = {v:12.6f}")

    print(f"\n  物理解释参考: K4={k[4]:.4f} (单电机铜损 3R≈{3*R_PH_REF:.2f}), "
          f"K5={k[5]:.6f} (铁损系数, 通常很小), K0={k[0]:.4f}")
    print(f"  固件修正项建议: CorrectionConstant = -3*K0 = {-3*k[0]:.3f}")

    print("\n" + "=" * 60)
    print("C 语言宏定义 (可直接替换 can_send_task.cpp 里的 poly_coeffs)")
    print("=" * 60)
    print(f"#define K0  {k[0]:.6f}f")
    print(f"#define K1  {k[1]:.6f}f")
    print(f"#define K2  {k[2]:.6f}f")
    print(f"#define K3  {k[3]:.6f}f")
    print(f"#define K4  {k[4]:.6f}f")
    print(f"#define K5  {k[5]:.6f}f")
    print("// P = K0 + K1*I + K2*w + K3*I*w + K4*I*I + K5*w*w")
    print("// float poly_coeffs[6] = {K0, K1, K2, K3, K4, K5};")
    if NUM_MOTORS > 1:
        print("// 整车: P_total = 4*K0 + K1*ΣI + K2*Σw + K3*Σ(I*w) + K4*Σ(I²) + K5*Σ(w²)")
        print("//       与固件 post_power 求和结构一致")

    # ---- 画图 ----
    plot_results(P, W, I, P_est, tgt, k, n_train, path)
    return k


def plot_results(P, W, I, P_est, tgt, k, n_train, path):
    """三合一图: 工况散点 / 预测vs实测 / 误差"""
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
    plt.rcParams['axes.unicode_minus'] = False
    P_pred = calc_power(k, W, I)
    save_dir = os.path.dirname(os.path.abspath(path))
    save_path = os.path.join(save_dir, 'power_predict_fit.png')

    fig, axes = plt.subplots(3, 1, figsize=(13, 12))

    # 图1: 工况覆盖散点 (所有电机合并)
    aw = np.concatenate([W[:, i] for i in range(NUM_MOTORS)])
    aI = np.concatenate([I[:, i] for i in range(NUM_MOTORS)])
    aP = np.tile(P, NUM_MOTORS)
    sc = axes[0].scatter(aw, aI, c=aP, s=2, cmap='jet')
    axes[0].set_xlabel('w (rad/s)')
    axes[0].set_ylabel('I (A)')
    axes[0].set_title(f'工况覆盖 (w-I 平面), 颜色=功率计 P (W), 共 {len(P)} 行 x {NUM_MOTORS} 电机')
    fig.colorbar(sc, ax=axes[0])

    # 图2: 预测 vs 实测 (时间轴), 标注验证集
    x = np.arange(len(P))
    axes[1].plot(x, P, 'b-', lw=0.6, alpha=0.7, label='功率计实测 P')
    axes[1].plot(x, P_pred, 'r--', lw=0.6, label='模型预测 P_pred')
    if P_est is not None and np.any(np.abs(P_est) > 0):
        axes[1].plot(x, P_est, 'g:', lw=0.6, alpha=0.8, label='固件原 P_est')
    axes[1].axvspan(n_train, len(P), color='orange', alpha=0.15, label='验证集')
    axes[1].set_xlabel('采样点')
    axes[1].set_ylabel('功率 (W)')
    axes[1].set_title('功率: 实测 vs 模型预测')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(alpha=0.3)

    # 图3: 误差
    err = P - P_pred
    axes[2].plot(x, err, 'g-', lw=0.5)
    axes[2].axhline(0, color='k', lw=0.5)
    axes[2].axvspan(n_train, len(P), color='orange', alpha=0.15)
    axes[2].set_xlabel('采样点')
    axes[2].set_ylabel('误差 (W)')
    axes[2].set_title(f'拟合误差 | RMSE={np.sqrt(np.mean(err**2)):.2f} W,  '
                      f'P90={np.quantile(np.abs(err), 0.9):.2f} W')
    axes[2].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"\n图表已保存: {save_path}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        CSV_PATH = sys.argv[1]
    df = load_data(CSV_PATH)
    P, W, I, tgt, P_est = extract(df)
    diagnostics(P, W, I)
    P, W, I, P_est, tgt = clean_data(P, W, I, P_est, tgt)
    if len(P) < 1000:
        sys.exit(f"错误: 有效数据只有 {len(P)} 点, 检查数据或放宽清洗阈值")
    fit_and_report(P, W, I, P_est, tgt, CSV_PATH)
