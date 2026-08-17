import pandas as pd
import numpy as np
import allantools
import matplotlib.pyplot as plt

# === Чтение и компенсация переполнения micros() ===
df = pd.read_csv('data.csv', header=None,
                  names=['t_us', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz'])

t_us = df['t_us'].to_numpy().astype(np.int64)
diffs = np.diff(t_us)

OVERFLOW = 2**32
n_overflows = (diffs < 0).sum()
diffs[diffs < 0] += OVERFLOW

t_us_fixed = np.concatenate(([t_us[0]], t_us[0] + np.cumsum(diffs)))
df['t_us'] = t_us_fixed  # ЗАМЕНЯЕМ старый столбец, а не добавляем новый

dt = diffs / 1e6
avg_rate = 1.0 / dt.mean()
print(f"Исправленная частота: {avg_rate:.2f} Гц, джиттер (std dt): {dt.std()*1000:.3f} мс")
print(f"Найдено переполнений micros(): {n_overflows}")

# Опционально сохранить очищенный файл — теперь ровно 10 столбцов, без путаницы
df.to_csv('data_fixed.csv', index=False, header=False)

# === Allan deviation по всем 6 осям ===
fig, axes = plt.subplots(2, 3, figsize=(15, 8))
axis_names = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
titles = ['Accel X', 'Accel Y', 'Accel Z', 'Gyro X', 'Gyro Y', 'Gyro Z']

for ax, name, title in zip(axes.flat, axis_names, titles):
    data = df[name].to_numpy()
    taus, adevs, errors, ns = allantools.oadev(data, rate=avg_rate, data_type="freq", taus="octave")

    ax.loglog(taus, adevs, 'o-', markersize=4)
    ax.set_xlabel('τ (с)')
    ax.set_ylabel('Allan deviation')
    ax.set_title(title)
    ax.grid(True, which="both", ls="--", alpha=0.5)

    ref_x = np.array([taus[0], taus[-1]])
    ref_y_white = adevs[0] * (ref_x / taus[0]) ** -0.5
    ax.loglog(ref_x, ref_y_white, '--', color='gray', alpha=0.4)

plt.tight_layout()
plt.savefig('allan_deviation_night.png', dpi=150)
plt.show()