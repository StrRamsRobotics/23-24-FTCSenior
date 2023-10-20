package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Motor {
    private DcMotorEx motor;
    private MotorThread thread;
    private final double kp = 0.5, ki = 0.1, kd = 0.2;
    private final double TICKS_PER_REV = 537.6;
    private final double deltaT = 0.001;
    public Motor(DcMotorEx motor) {
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // angular velocity
    public void setVelocity(double velocity) {
        thread = new MotorThread((int)(velocity));
        thread.start();
    }
    private class MotorThread extends Thread {
        private int ticks;
        private double target;
        private double prev_error = 0;
        private double sum = 0;
        public MotorThread(int target) {
            this.target = target;
        }
        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                    double speed = (double)(motor.getCurrentPosition() - ticks) / (deltaT * TICKS_PER_REV);
                    double error = target - speed;
                    double power = clamp(kp * error + ki * sum + (error - prev_error) * kd, 1, -1); // formula
                    // proportional (error) integral (sum is integral) derivative (difference between speed)
                    motor.setPower(power);
                    ticks = motor.getCurrentPosition();
                    prev_error = speed;
                    sum += speed;
                    Thread.sleep((int)deltaT * 1000);
                }
            }
            catch (Exception e) { // needed for thread

            }
        }

        public double clamp(double value, double max, double min) {
            return Math.max(Math.min(value, max), min);
        }
    }
}
