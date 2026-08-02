import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# 1. Генерация вершин куба (от -1 до 1)
def get_cube_vertices():
    return np.array([
        [-1, -1, -1],
        [ 1, -1, -1],
        [ 1,  1, -1],
        [-1,  1, -1],
        [-1, -1,  1],
        [ 1, -1,  1],
        [ 1,  1,  1],
        [-1,  1,  1]
    ])

# 2. Определение ребер куба (индексы вершин, которые нужно соединить)
edges = [
    (0, 1), (1, 2), (2, 3), (3, 0),  # Нижняя грань
    (4, 5), (5, 6), (6, 7), (7, 4),  # Верхняя грань
    (0, 4), (1, 5), (2, 6), (3, 7)   # Вертикальные ребра
]

# 3. Матрицы поворота вокруг осей X, Y, Z с помощью NumPy
def get_rotation_matrix(alpha, beta, gamma):
    # Перевод градусов в радианы
    a, b, g = np.radians([alpha, beta, gamma])
    
    # Матрица поворота вокруг X
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(a), -np.sin(a)],
        [0, np.sin(a), np.cos(a)]
    ])
    
    # Матрица поворота вокруг Y
    Ry = np.array([
        [np.cos(b), 0, np.sin(b)],
        [0, 1, 0],
        [-np.sin(b), 0, np.cos(b)]
    ])
    
    # Матрица поворота вокруг Z
    Rz = np.array([
        [np.cos(g), -np.sin(g), 0],
        [np.sin(g), np.cos(g), 0],
        [0, 0, 1]
    ])
    
    # Перемножение матриц (итоговый поворот)
    return Rz @ Ry @ Rx

# 4. Настройка графического окна Matplotlib
fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.25)  # Оставляем место внизу для слайдеров

# Базовые вершины куба
vertices = get_cube_vertices()
lines = []

# Инициализация линий на графике
for edge in edges:
    line, = ax.plot([], [], [], color='blue', lw=2)
    lines.append(line)

# Настройка осей координат
def reset_axes():
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Интерактивный 3D Куб (NumPy + Matplotlib)")

reset_axes()

# 5. Функция обновления графика при изменении слайдеров
def update(val):
    alpha = slider_x.val
    beta = slider_y.val
    gamma = slider_z.val
    
    # Получаем матрицу поворота
    R = get_rotation_matrix(alpha, beta, gamma)
    
    # Поворачиваем все вершины куба: умножаем матрицу на вектор каждой вершины
    # Матрица (3,3) умножается на транспонированный массив вершин (3,8)
    rotated_vertices = (R @ vertices.T).T
    
    # Перерисовываем ребра по новым координатам
    for line, edge in zip(lines, edges):
        p1, p2 = rotated_vertices[edge[0]], rotated_vertices[edge[1]]
        line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
        line.set_3d_properties([p1[2], p2[2]])
    
    fig.canvas.draw_idle()

# 6. Создание слайдеров для управления углами
ax_color = 'lightgoldenrodyellow'
ax_x = plt.axes([0.2, 0.15, 0.65, 0.03], facecolor=ax_color)
ax_y = plt.axes([0.2, 0.10, 0.65, 0.03], facecolor=ax_color)
ax_z = plt.axes([0.2, 0.05, 0.65, 0.03], facecolor=ax_color)

slider_x = Slider(ax_x, 'Угол X (°)', 0, 360, valinit=30)
slider_y = Slider(ax_y, 'Угол Y (°)', 0, 360, valinit=30)
slider_z = Slider(ax_z, 'Угол Z (°)', 0, 360, valinit=0)

# Привязка функции обновления к слайдерам
slider_x.on_changed(update)
slider_y.on_changed(update)
slider_z.on_changed(update)

# Первый запуск для отображения куба в начальном положении
update(None)

plt.show()
