#!/usr/bin/env python3
import os
import pandas as pd
import matplotlib.pyplot as plt
import warnings

warnings.filterwarnings("ignore", category=UserWarning)

def load_and_compute(csv_path):
    """
    CSV 파일을 읽어서 t_recv_ns - t_sent_ns 차이를 계산하고,
    Nanosecond 단위를 밀리초(ms)로 변환하여 latency_ms 컬럼을 추가합니다.
    """
    df = pd.read_csv(csv_path)
    df["latency_ms"] = (df["t_recv_ns"] - df["t_sent_ns"]).clip(lower=0) / 1e6
    return df

def summarize(df):
    """
    latency_ms 시리즈에 대해 평균, 최소, 최대, 표준편차를 계산하고,
    스파이크 임계값(mean + 3*std)을 구합니다.
    이 임계값을 넘는 샘플은 spikes에 담아 반환합니다.
    """
    stats = df["latency_ms"].agg(['mean','min','max','std'])
    threshold = stats['mean'] + 3 * stats['std']
    spikes = df[df["latency_ms"] > threshold]
    stats = stats.rename({
        'mean': 'average_ms',
        'min':  'min_ms',
        'max':  'max_ms',
        'std':  'std_ms'
    })
    return stats, threshold, spikes

def plot_latency(df, stats, threshold, out_path):
    """
    시퀀스별 레이턴시를 플롯하고 요약 통계치를 그림 내부에 텍스트 박스로 추가합니다.
    """
    plt.figure(figsize=(12,6))
    ax = plt.gca()
    x = df["seq"].to_numpy()
    y = df["latency_ms"].to_numpy()

    ax.plot(x, y, label="Latency (ms)")
    ax.axhline(threshold, linestyle='--', color='red',
               label=f"Spike Threshold ({threshold:.3f} ms)")
    ax.axhline(0.1, linestyle=':', color='blue',
               label="Hard RTOS Threshold (0.1 ms)")
    ax.axhline(1.0, linestyle='-.', color='green',
               label="Soft RTOS Threshold (1 ms)")
    ax.scatter(x[y > threshold], y[y > threshold],
               color='red', s=10, label="Spikes")

    # 텍스트 박스
    textstr = (
        f"Average: {stats['average_ms']:.3f} ms\n"
        f"Min:     {stats['min_ms']:.3f} ms\n"
        f"Max:     {stats['max_ms']:.3f} ms\n"
        f"Stddev:  {stats['std_ms']:.3f} ms\n"
        f"Spikes:  {len(df[df['latency_ms'] > threshold])}"
    )
    props = dict(boxstyle='round', facecolor='white', alpha=0.7)
    ax.text(1.02, 0.5, textstr, transform=ax.transAxes, fontsize=10,
            verticalalignment='center', bbox=props)

    ax.set_xlabel("Sequence")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Latency Over Sequence with RTOS Thresholds")
    ax.grid(True)
    ax.legend(loc='upper left', bbox_to_anchor=(0,1))
    plt.tight_layout()
    plt.savefig(out_path, bbox_inches='tight')
    plt.show()

def main():
    csv_path = "QNX_reserve_onpriority_c12_253253_mlock_sub_log_size32_hz100_count10000_qosKEEP_LAST_RELIABLE.csv"
    df = load_and_compute(csv_path)
    stats, threshold, spikes = summarize(df)

    print("📊 Latency Summary (ms):")
    print(stats.to_string())
    print(f"\nSpike threshold: {threshold:.4f} ms")
    print(f"Spike count: {len(spikes)}")
    if not spikes.empty:
        print("\nFirst few spike entries:")
        print(spikes[['seq','latency_ms']].head().to_string(index=False))

    
    base_name = os.path.splitext(csv_path)[0]
    out_path = f"{base_name}.png"
    plot_latency(df, stats, threshold, out_path)
    
if __name__ == "__main__":
    main()
