package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

public class PDController {
    private final PIDCoefficients coefficients; //pass by ref allows ftcdashboard config to work
    private double prevError;
    private double prevTime;

    public PDController(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
        prevError = 0;
        prevTime = 0;
    }

    public double update(double error) {
        double currentTime = System.currentTimeMillis();
        double dt = currentTime - prevTime;
        double de = error - prevError;
        double kP = coefficients.kP;
        double kD = coefficients.kD;
        double output = kP * error + kD * de / dt;
        prevError = error;
        prevTime = currentTime;
        return output;
    }
}
