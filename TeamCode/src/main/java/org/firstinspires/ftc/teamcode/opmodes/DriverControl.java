package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.Outtake;

@TeleOp
public class DriverControl extends LinearOpMode {
    static Outtake slides;
    static int slidePosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        slides = new Outtake(hardwareMap);
        waitForStart();
        Gamepad chassisControl = this.gamepad1;
        Gamepad armControl = this.gamepad2;
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        while (opModeIsActive()) {
            double lx = chassisControl.left_stick_x;
            double ly = -chassisControl.left_stick_y;
            double rx = chassisControl.right_stick_x;

            if (Math.abs(lx)<0.1) lx = 0;
            if (Math.abs(ly)<0.1) ly = 0;
            if (Math.abs(rx)<0.1) rx = 0;
            lx = smooth(lx);
            ly = smooth(ly);
            rx = smooth(rx); //rx+ry reverses when plugged in laptop

            telemetry.addData("left x", lx);
            telemetry.addData("left y", ly);
            telemetry.addData("right x", rx);
            telemetry.addData("intake: ", (chassisControl.right_bumper?0.9:0)-(chassisControl.left_bumper?0.9:0));
            telemetry.update();

            Init.frontLeftDrive.setPower(Math.min(1, Math.max(-1, lx + ly + rx)));
            Init.frontRightDrive.setPower(Math.min(1, Math.max(-1, -lx + ly - rx)));
            Init.backLeftDrive.setPower(Math.min(1, Math.max(-1, -lx + ly + rx)));
            Init.backRightDrive.setPower(Math.min(1, Math.max(-1, lx + ly - rx)));

            Init.leftClimb.setPower((chassisControl.y?1:0)-(chassisControl.a?1:0));
            Init.rightClimb.setPower((chassisControl.y?1:0)-(chassisControl.a?1:0));
            telemetry.addData("left climb", Init.leftClimb.getCurrentPosition());
            telemetry.addData("right climb", Init.rightClimb.getCurrentPosition());
            telemetry.update();

            Init.intakeTilt.setPosition(1-chassisControl.left_trigger);
            Init.intake.setPower((chassisControl.right_bumper?0.7:0)-(chassisControl.left_bumper?0.7:0));

            slidePosition += 10*(int)armControl.left_stick_y;
            slides.runSlide(slidePosition, 0.7);
            if (armControl.a) Init.outtake.setPosition(Outtake.EXTENSION_POS);
            else Init.outtake.setPosition(Outtake.RETRACTION_POS); //4101
        }
    }
    private double smooth(double in) {
        return in*in*in; //more non-linear than 2^x
    }
}