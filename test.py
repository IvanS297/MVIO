import serial, math, sys
s = serial.Serial(sys.argv[1], 115200, timeout=1)
angle = 0.0
t0 = t_prev = None
n = 0
dts = []
while True:
    try:
        v = [float(x) for x in s.readline().decode().split(",")]
    except ValueError:
        continue
    if len(v) != 10:
        continue
    t = v[0] / 1000.0          # мс -> с
    if t0 is None:
        t0 = t_prev = t
        continue
    dt = t - t_prev
    t_prev = t
    angle += v[6] * dt
    dts.append(dt)
    n += 1
    if n % 50 == 0:
        print(f"{n/(t-t0):6.1f} Гц  dt min/max: {min(dts)*1000:.0f}/{max(dts)*1000:.0f} мс  угол: {math.degrees(angle):7.1f}°")
        dts.clear()
