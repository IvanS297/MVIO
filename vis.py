#!/usr/bin/python3

import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

# --- 1. Парсинг Roll, Pitch, Yaw из файла out.txt ---
rolls, pitches, yaws = [], [], []

# Регулярное выражение только для строки с углами
rpy_pattern = re.compile(r"Roll:\s*([\d.-]+)\s*Pitch:\s*([\d.-]+)\s*Yaw:\s*([\d.-]+)")

with open("out.txt", "r") as f:
    for line in f:
        rpy_match = rpy_pattern.search(line)
        if rpy_match:
            rolls.append(float(rpy_match.group(1)))
            pitches.append(float(rpy_match.group(2)))
            yaws.append(float(rpy_match.group(3)))

rolls = np.array(rolls)
pitches = np.array(pitches)
yaws = np.array(yaws)

num_frames = len(rolls)
if num_frames == 0:
    raise ValueError("Не удалось найти строки с углами Roll, Pitch, Yaw в файле out.txt")

# Создаем индекс кадров для оси X на графике
time_index = np.arange(num_frames)

# --- 2. Настройка интерфейса ---
fig = plt.figure(figsize=(12, 6))

# Левая половина: График изменения углов во времени
ax_rpy = fig.add_subplot(1, 2, 1)
# Правая половина: Вращающаяся 3D-модель (оси)
ax_3d = fig.add_subplot(1, 2, 2, projection='3d')

# Отрисовка линий на графике
ax_rpy.plot(time_index, rolls, 'r-', label='Roll')
ax_rpy.plot(time_index, pitches, 'g-', label='Pitch')
ax_rpy.plot(time_index, yaws, 'b-', label='Yaw')
ax_rpy.set_title("Углы ориентации (Градусы)")
ax_rpy.set_xlabel("Номер измерения (кадр)")
ax_rpy.legend()
ax_rpy.grid(True)

# Базовые единичные векторы для отрисовки объекта в 3D
basis = np.eye(3)

# --- 3. Функция обновления кадра ---
def update(frame):
    ax_3d.cla()
    ax_3d.set_xlim([-1.5, 1.5])
    ax_3d.set_ylim([-1.5, 1.5])
    ax_3d.set_zlim([-1.5, 1.5])
    ax_3d.set_title(f"3D Ориентация (Кадр {frame+1}/{num_frames})")
    
    # Строим матрицу поворота по углам Эйлера (в градусах)
    r = R.from_euler('xyz', [rolls[frame], pitches[frame], yaws[frame]], degrees=True)
    current_rot = r.as_matrix()
    
    # Поворачиваем наши виртуальные оси
    rotated_basis = current_rot @ basis
    
    # Рисуем повернутые оси: X (красная), Y (зеленая), Z (синяя)
    ax_3d.quiver(0, 0, 0, rotated_basis[0, 0], rotated_basis[1, 0], rotated_basis[2, 0], color='r', length=1.0)
    ax_3d.quiver(0, 0, 0, rotated_basis[0, 1], rotated_basis[1, 1], rotated_basis[2, 1], color='g', length=1.0)
    ax_3d.quiver(0, 0, 0, rotated_basis[0, 2], rotated_basis[1, 2], rotated_basis[2, 2], color='b', length=1.0)
    
    # Добавляем подписи к осям для наглядности
    ax_3d.text(rotated_basis[0, 0]*1.1, rotated_basis[1, 0]*1.1, rotated_basis[2, 0]*1.1, 'X', color='r')
    ax_3d.text(rotated_basis[0, 1]*1.1, rotated_basis[1, 1]*1.1, rotated_basis[2, 1]*1.1, 'Y', color='g')
    ax_3d.text(rotated_basis[0, 2]*1.1, rotated_basis[1, 2]*1.1, rotated_basis[2, 2]*1.1, 'Z', color='b')

    # Вертикальный маркер текущего кадра на 2D графике
    if len(ax_rpy.lines) > 3:
        ax_rpy.lines[-1].remove()
    ax_rpy.axvline(x=frame, color='black', linestyle='--', alpha=0.6)

# Запуск циклической анимации (интервал 250 мс)
ani = FuncAnimation(fig, update, frames=num_frames, interval=10, repeat=True)
plt.tight_layout()
plt.show()
