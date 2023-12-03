package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class DriverControl extends LinearOpMode {
    static int slidePosition = 0;
    static int climbPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        Gamepad chassisControl = this.gamepad1;
        Gamepad armControl = this.gamepad2;
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        while (opModeIsActive()) {
            double lx = chassisControl.left_stick_x;
            double ly = -chassisControl.left_stick_y;
            double rx = chassisControl.right_stick_x;

            if (Math.abs(lx)<0.02) lx = 0;
            if (Math.abs(ly)<0.02) ly = 0;
            if (Math.abs(rx)<0.02) rx = 0;
            lx = smooth(lx);
            ly = smooth(ly);
            rx = smooth(rx); //rx+ry reverses when plugged in laptop

            telemetry.addData("left x", lx);
            telemetry.addData("left y", ly);
            telemetry.addData("right x", rx);
            telemetry.addData("intake: ", (chassisControl.right_bumper?0.9:0)-(chassisControl.left_bumper?0.9:0));
            telemetry.addData("rightClimb: ", Init.rightClimb.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("leftClimb: ", Init.rightClimb.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            drive.update();

            telemetry.addData("left climb", Init.leftClimb.getCurrentPosition());
            telemetry.addData("right climb", Init.rightClimb.getCurrentPosition());
            telemetry.update();

            Init.intakeTilt.setPosition(1-chassisControl.left_trigger);
            Init.intake.setPower((chassisControl.right_bumper?0.7:0)-(chassisControl.left_bumper?0.7:0));

            slidePosition -= 20*armControl.left_stick_y;
            climbPosition += (chassisControl.a?10:0)-(chassisControl.y?10:0);
            Init.out.runSlide(slidePosition, 1);
            Init.leftClimb.setPower((chassisControl.a?0.2:0)-(chassisControl.y?0.2:0));
            Init.rightClimb.setPower((chassisControl.a?0.4:0)-(chassisControl.y?0.4:0));
//            Init.climb.runClimb(climbPosition, 1);
//            Init.leftClimb.setPower(climbPosition/30);
//            Init.out.runServo(armControl.a); //4101
            Init.outtake.setPosition(armControl.a?Init.out.EXTENSION_POS:Init.out.RETRACTION_POS);
        }
    }
    private double smooth(double in) {
        return in*in*in; //more non-linear than 2^x
    }
}