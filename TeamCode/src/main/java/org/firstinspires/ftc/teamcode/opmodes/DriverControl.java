package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.Executors;

@TeleOp
public class DriverControl extends LinearOpMode {
    private volatile boolean planing= false;
    int slidePosition = 0; //static = persist between inits
    int climbPosition = 0;
    static final int slideMaxPos = 1900;

    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (!isStarted()) {
            telemetry.addData("left slide", Init.leftSlide.getCurrentPosition());
            telemetry.addData("right slide", Init.rightSlide.getCurrentPosition());
            telemetry.addData("slide", slidePosition);
            telemetry.update();
        }
        waitForStart();
        Gamepad chassisControl = this.gamepad1;
        Gamepad armControl = this.gamepad2;
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
//            telemetry.addData("intake: ", (chassisControl.right_bumper?0.9:0)-(chassisControl.left_bumper?0.9:0));
            telemetry.update();

            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            drive.update();

            telemetry.addData("left climb", Init.leftClimb.getCurrentPosition());
            telemetry.addData("right climb", Init.rightClimb.getCurrentPosition());
            telemetry.addData("left slide", Init.leftSlide.getCurrentPosition());
            telemetry.addData("right slide", Init.rightSlide.getCurrentPosition());
            telemetry.update();

//            Init.intakeTilt.setPosition(1-armControl.left_trigger);
            if (armControl.dpad_up) Init.intakeTilt.setPosition(0.91);
            else if (armControl.dpad_down) Init.intakeTilt.setPosition(1);
            Init.intake.setPower((armControl.left_bumper?0.7:0)-(armControl.right_bumper?0.7:0));


            if (armControl.y) {
                Init.out.runSlide(slideMaxPos, 0.7);
                slidePosition = slideMaxPos;
            }
            else if (armControl.a) {
                Init.out.runSlide(0, 0.7);
                slidePosition = 0;
            } else {
                slidePosition += 20*(Math.abs(armControl.left_trigger-armControl.right_trigger)>0.1?armControl.left_trigger-armControl.right_trigger:0);
                slidePosition = Math.max(0, Math.min(slideMaxPos, slidePosition));
                Init.out.runSlide(slidePosition, 1);
            }
            if (!planing) {
                Init.leftClimb.setPower(chassisControl.left_trigger - (chassisControl.left_bumper ? 1 : 0));
                Init.rightClimb.setPower(chassisControl.right_trigger - (chassisControl.right_bumper ? 1 : 0));
            }
            Init.leftOuttake.setPosition(armControl.b?Init.out.EXTENSION_POS:Init.out.RETRACTION_POS);
            Init.rightOuttake.setPosition(armControl.b?Init.out.EXTENSION_POS:Init.out.RETRACTION_POS);
//            Servo plane = hardwareMap.get(Servo.class, "plane");
            if (chassisControl.dpad_down&&chassisControl.a&&!planing) {
                planing=true;
                Executors.newSingleThreadExecutor().execute(()->{
                    Init.rightClimb.setTargetPosition(-7600);
                    Init.rightClimb.setPower(1);
                    Init.rightClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (Init.rightClimb.isBusy()) {
                        try {
                            Thread.sleep(10);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    Init.plane.setPosition(0.43);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    Init.rightClimb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    planing = false;
                });

            } else if (!planing){
                Init.plane.setPosition(0.87);
            }
        }
    }
    private double smooth(double in) {
        return in*in*in; //more non-linear than 2^x
    }
}