package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drive Control Switch")
public class JoystickMovementOpMode extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private double speedMultiplier = 0.8; // Adjust this to control the overall speed of the robot
    private boolean isMecanumDrive = true; // Initial drive mode is mecanum drive

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        backLeftMotor = hardwareMap.dcMotor.get("back_left_motor");
        backRightMotor = hardwareMap.dcMotor.get("back_right_motor");

        // Reverse the direction of right motors if necessary
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double drive;
            double strafe;
            double rotate;

            if (isMecanumDrive) {
                drive = -gamepad1.left_stick_y; // Negative sign to invert joystick direction
                strafe = gamepad1.left_stick_x;
                rotate = gamepad1.right_stick_x;
            } else {
                drive = -gamepad1.left_stick_y; // Negative sign to invert joystick direction
                strafe = 0.0;
                rotate = gamepad1.right_stick_x;
            }

            // Calculate motor powers based on the selected drive mode
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;

            if (isMecanumDrive) {
                frontLeftPower = Range.clip(drive + strafe + rotate, -1.0, 1.0);
                frontRightPower = Range.clip(drive - strafe - rotate, -1.0, 1.0);
                backLeftPower = Range.clip(drive - strafe + rotate, -1.0, 1.0);
                backRightPower = Range.clip(drive + strafe - rotate, -1.0, 1.0);
            } else {
                frontLeftPower = Range.clip(drive + rotate, -1.0, 1.0);
                frontRightPower = Range.clip(drive + rotate, -1.0, 1.0);
                backLeftPower = Range.clip(drive + rotate, -1.0, 1.0);
                backRightPower = Range.clip(drive + rotate, -1.0, 1.0);
            }

            // Apply speed multiplier
            frontLeftPower *= speedMultiplier;
            frontRightPower *= speedMultiplier;
            backLeftPower *= speedMultiplier;
            backRightPower *= speedMultiplier;

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            // Check if gamepad1.a is pressed to toggle between mecanum and tank drive
            if (gamepad1.a) {
                isMecanumDrive = !isMecanumDrive;
                sleep(200); // Delay to prevent multiple toggles from a single press
            }

            telemetry.addData("Drive Mode", isMecanumDrive ? "Mecanum" : "Tank");
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();
        }
    }
}
