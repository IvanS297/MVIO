#include <sciplot/sciplot.hpp>
#include <cmath>

int main() {
    using namespace sciplot;

    // Данные спирали
    Vec z = linspace(0, 100, 2000);
    std::vector<double> x, y;
    for (auto val : z) {
        x.push_back(std::cos(val / 2));
        y.push_back(std::sin(val / 2));
    }

    // Создаём 3D-график
    Plot3D plot;
    plot.drawCurve(x, y, z).lineColor("orange").label("helix");
    plot.xlabel("x");
    plot.ylabel("y");
    plot.zlabel("z");

    // Помещаем в фигуру и холст
    Figure fig = {{plot}};
    Canvas canvas = {{fig}};

    // Показываем окно (интерактивное, вращается мышью)
    canvas.show();

    // Сохраняем опционально
    // canvas.save("helix.pdf");

    return 0;
}