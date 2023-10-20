package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Example Autonomous")
public class ExampleAutonomousOpMode extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private static final double COUNTS_PER_REV = 1120; // Number of encoder counts per revolution
    private static final double WHEEL_DIAMETER_INCHES = 4; // Diameter of the wheels in inches
    private static final double DRIVE_GEAR_RATIO = 1; // Gear ratio of the motor
    private static final double COUNTS_PER_INCH = (COUNTS_PER_REV * DRIVE_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // Move forward for 24 inches
        driveForward(24);

        // Pause for 1 second
        sleep(1000);

        // Turn right for 90 degrees
        turnRight(90);

        // Pause for 1 second
        sleep(1000);

        // Move backward for 12 inches
        driveBackward(12);
    }

    private void driveForward(double inches) {
        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(0.5);

        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            // Wait for motors to reach target position
            idle();
        }

        setMotorPower(0);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveBackward(double inches) {
        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        leftMotor.setTargetPosition(-targetPosition);
        rightMotor.setTargetPosition(-targetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(-0.5);

        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            // Wait for motors to reach target position
            idle();
        }

        setMotorPower(0);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnRight(double degrees) {
        double inches = degrees * Math.PI * 12 / 360; // Assuming a 12-inch wheelbase

        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(-targetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(0.5);

        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            // Wait for motors to reach target position
            idle();
        }

        setMotorPower(0);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    private void setMotorPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}
