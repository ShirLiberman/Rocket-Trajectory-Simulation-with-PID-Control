#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>

class PIDController {
private:
    double kp;
    double ki;
    double kd;
    double prevError;
    double integral;

public:
    PIDController(double p, double i, double d) : kp(p), ki(i), kd(d), prevError(0), integral(0) {}

    double update(double error, double dt) {
        double derivative = (error - prevError) / dt;
        integral += error * dt;

        double output = kp * error + ki * integral + kd * derivative;
        prevError = error;
        return output;
    }
};

class RocketSimulation {
private:
    const double gravity = -9.81;  // m/s^2
    const double dt = 0.1;         // seconds
    PIDController yawController;
    const double PI = 3.14159265359;

    struct Point3D {
        double x, y, z;
        Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    };

    // Helper function to convert degrees to radians
    double toRadians(double degrees) {
        return degrees * PI / 180.0;
    }

    // Helper function to normalize angle to [-180, 180]
    double normalizeAngle(double degrees) {
        degrees = fmod(degrees, 360.0);
        if (degrees > 180.0) {
            degrees -= 360.0;
        }
        else if (degrees < -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

public:
    RocketSimulation() : yawController(0.4, 0.005, 0.045) {}

    Point3D calculateImpactPoint(double x0, double y0, double z0, double vx0, double vy0, double vz0) {
        double a = 0.5 * gravity;
        double b = vy0;
        double c = y0;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            std::cout << "Warning: Trajectory will not hit ground with these parameters!" << std::endl;
            return Point3D(0, 0, 0);
        }

        double timeToImpact1 =  (-b + sqrt(discriminant)) / (2 * a);
        double timeToImpact2 =  (-b - sqrt(discriminant)) / (2 * a);
        double timeToImpact;

        // Handle the case when y0 is 0
        if (std::abs(y0) < 1e-9) {
            timeToImpact = -vy0 / gravity;
        }
        else {
            if (timeToImpact1 >= 0 && timeToImpact2 >= 0) {
                timeToImpact = std::min(timeToImpact1, timeToImpact2);
            }
            else if (timeToImpact1 >= 0) {
                timeToImpact = timeToImpact1;
            }
            else if (timeToImpact2 >= 0) {
                timeToImpact = timeToImpact2;
            }
            else {
                std::cout << "Warning: Both times to impact are negative!" << std::endl;
                return Point3D(0, 0, 0);
            }
        }

        double xImpact = (x0 + vx0 * timeToImpact) * 2;
        double zImpact = (z0 + vz0 * timeToImpact) * 2;

        std::cout << "Estimated time to impact: " << std::fixed << std::setprecision(2)
            << timeToImpact * 2 << " seconds" << std::endl;

        return Point3D(xImpact, 0, zImpact);
    }

    void runSimulation(double x0, double y0, double z0,
        double initialSpeed, double launchAngle,
        double initialYaw, double targetYaw) {
        // Convert angles to radians
        double yawRad = toRadians(targetYaw);

        // Calculate initial velocities based on speed, launch angle, and initial yaw
        double launchAngleRad = toRadians(launchAngle);
        double vx0 = initialSpeed * cos(launchAngleRad) * cos(yawRad);
        double vy0 = initialSpeed * sin(launchAngleRad);
        double vz0 = initialSpeed * cos(launchAngleRad) * sin(yawRad);

        // Calculate the estimated impact point
        Point3D impactPoint = calculateImpactPoint(x0, y0, z0, vx0, vy0, vz0);
        std::cout << "\nEstimated Impact Point:" << std::endl;
        std::cout << "X: " << impactPoint.x << std::endl;
        std::cout << "Y: 0.00" << std::endl;
        std::cout << "Z: " << impactPoint.z << std::endl;

        std::cout << "\nStarting simulation in 3 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));

        double x = x0, y = y0, z = z0;
        double totalVelocity = initialSpeed;
        double currentYaw = initialYaw;
        double t = 0;
        double vx = vx0, vy = vy0, vz = vz0;

        bool reachedApex = false;

        std::cout << "\nTime(s) | X | Y | Z | VX | VY | VZ | YAW | Speed" << std::endl;
        std::cout << std::string(60, '-') << std::endl;

        while (y >= 0 || !reachedApex) {
            // Calculate yaw error and update yaw
            double yawError = normalizeAngle(targetYaw - currentYaw);
            double yawCorrection = yawController.update(yawError, dt);
            currentYaw = normalizeAngle(currentYaw + yawCorrection);

            // Convert current yaw to radians
            double currentYawRad = toRadians(currentYaw);

            // Calculate horizontal velocity components based on current yaw
            double horizontalSpeed = sqrt(vx * vx + vz * vz);
            vx = horizontalSpeed * cos(currentYawRad);
            vz = horizontalSpeed * sin(currentYawRad);

            // Update position
            x += vx * dt;
            y += vy * dt;
            z += vz * dt;

            // Update vertical velocity (affected by gravity)
            vy += gravity * dt;

            // Calculate total velocity
            totalVelocity = sqrt(vx * vx + vy * vy + vz * vz);

            // Print rocket state
            std::cout << std::fixed << std::setprecision(2)
                << t << " | "
                << x << " | "
                << y << " | "
                << z << " | "
                << vx << " | "
                << vy << " | "
                << vz << " | "
                << currentYaw << " | "
                << totalVelocity << std::endl;

            t += dt;

            // Check if the rocket has reached its peak height and is descending
            if (vy < 0) {
                reachedApex = true;
            }
        }

        std::cout << "\nThe rocket has hit the ground!" << std::endl;
    }
};

int main() {
    double x0, y0, z0, initialSpeed, launchAngle, initialYaw, targetYaw;

    std::cout << "Enter initial X position: ";
    std::cin >> x0;
    std::cout << "Enter initial Y position: ";
    std::cin >> y0;
    std::cout << "Enter initial Z position: ";
    std::cin >> z0;

    std::cout << "Enter initial speed (m/s): ";
    std::cin >> initialSpeed;
    std::cout << "Enter launch angle (degrees from horizontal): ";
    std::cin >> launchAngle;
    std::cout << "Enter initial YAW angle (degrees): ";
    std::cin >> initialYaw;
    std::cout << "Enter target YAW angle (degrees): ";
    std::cin >> targetYaw;

    RocketSimulation sim;
    sim.runSimulation(x0, y0, z0, initialSpeed, launchAngle, initialYaw, targetYaw);

    return 0;
}