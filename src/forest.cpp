#include <iostream>
#include <vector>
#include <functional>

using namespace std;

// Define the system of equations as a function
void model(const vector<double> &y, vector<double> &dydt, double t, const vector<double> &params)
{
    double B = y[0], r = y[1], I = y[2];
    double K = params[0], h1 = params[1], mu1 = params[2], r0 = params[3],
           rho = params[4], mu2 = params[5], h2 = params[6];

    dydt[0] = r * B * (1 - B / K) + h1 * B - mu1 * I * B; // dB/dt
    dydt[1] = r0 - rho * r;                               // dr/dt
    dydt[2] = mu2 * I * (B / (1 + B)) - h2 * I;           // dI/dt
}

// Implement the Runge-Kutta 4th order method
void runge_kutta_4(function<void(const vector<double> &, vector<double> &, double, const vector<double> &)> f,
                   vector<double> &y, double t0, double tf, double dt, const vector<double> &params)
{
    int steps = (int)((tf - t0) / dt);
    vector<double> dydt(y.size()), k1(y.size()), k2(y.size()), k3(y.size()), k4(y.size()), temp(y.size());

    for (int i = 0; i <= steps; i++)
    {
        double t = t0 + i * dt;
        cout << t << "," << y[0] << "," << y[1] << "," << y[2] << endl;

        f(y, dydt, t, params); // Calculate derivatives

        for (int j = 0; j < y.size(); j++)
            k1[j] = dt * dydt[j];
        for (int j = 0; j < y.size(); j++)
            temp[j] = y[j] + 0.5 * k1[j];
        f(temp, dydt, t + 0.5 * dt, params);

        for (int j = 0; j < y.size(); j++)
            k2[j] = dt * dydt[j];
        for (int j = 0; j < y.size(); j++)
            temp[j] = y[j] + 0.5 * k2[j];
        f(temp, dydt, t + 0.5 * dt, params);

        for (int j = 0; j < y.size(); j++)
            k3[j] = dt * dydt[j];
        for (int j = 0; j < y.size(); j++)
            temp[j] = y[j] + k3[j];
        f(temp, dydt, t + dt, params);

        for (int j = 0; j < y.size(); j++)
            k4[j] = dt * dydt[j];
        for (int j = 0; j < y.size(); j++)
            y[j] += (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]) / 6.0;
    }
}

int main()
{
    // Define parameters
    vector<double> params = {400, 0.9, 0.1, 0.2, 0.1, 0.3, 0.25};
    vector<double> y = {0.4, 0.2, 0.2}; // Initial conditions: B0, r0, I0
    double t0 = 0.0, tf = 400.0, dt = 0.1;

    // Run the simulation
    runge_kutta_4(model, y, t0, tf, dt, params);

    return 0;
}