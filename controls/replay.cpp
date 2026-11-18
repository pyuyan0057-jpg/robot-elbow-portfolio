// Sampled joint controller. Replay rows are recorded inputs, never reference torques.
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

enum class Mode { pd, pd_gravity, pid_gravity };

struct Controller {
    Mode mode;
    double integral = 0.0;
    double previous_angle = 0.0;
    double velocity = 0.0;
    static constexpr double dt = 0.001;
    static constexpr double kp = 18.0, kd = 1.8, ki = 20.0;

    void tick(double target, double target_speed, double angle,
              double gravity_sin, double gravity_cos, double limit) {
        const double alpha = -std::expm1(-dt / 0.006);
        velocity += alpha * ((angle - previous_angle) / dt - velocity);
        previous_angle = angle;
        const double error = target - angle;
        const double feedforward = mode == Mode::pd ? 0.0 :
            gravity_sin * std::sin(angle) + gravity_cos * std::cos(angle);
        const double base = kp * error + kd * (target_speed - velocity) + feedforward;
        const double next = mode == Mode::pid_gravity ? integral + dt * error : 0.0;
        const double tentative = base + ki * next;
        if (!(std::abs(tentative) > limit && tentative * error > 0.0))
            integral = next;
        const double request = base + ki * integral;
        std::cout << velocity << ' ' << request << ' '
                  << std::clamp(request, -limit, limit) << '\n';
    }
};

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    const std::string mode = argv[1];
    if (mode != "PD" && mode != "PDG" && mode != "PIDG") return 2;
    Controller controller{mode == "PD" ? Mode::pd :
                          mode == "PDG" ? Mode::pd_gravity : Mode::pid_gravity};
    std::cout << std::setprecision(17);
    double target, speed, angle, gsin, gcos, limit;
    std::size_t rows = 0;
    while (std::cin >> target) {
        if (!(std::cin >> speed >> angle >> gsin >> gcos >> limit) || limit <= 0)
            return 2;
        controller.tick(target, speed, angle, gsin, gcos, limit);
        ++rows;
    }
    return std::cin.eof() && rows ? 0 : 2;
}
