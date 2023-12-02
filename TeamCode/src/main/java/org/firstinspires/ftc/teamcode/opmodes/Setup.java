package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.Outtake;
@Config
@TeleOp
public class Setup extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        waitForStart();
        Init.rightClimb.setPower(0.2);
        Init.leftClimb.setPower(0.2);
        Init.rightClimb.setTargetPosition(-3258);
        Init.leftClimb.setTargetPosition(-3258);
        Init.rightClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Init.leftClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Init.rightClimb.isBusy()) {
            Thread.sleep(10);
        }
        Init.leftClimb.setPower(0);
        Init.intakeTilt.setPosition(0);
        Init.outtake.setPosition(Outtake.RETRACTION_POS);
    }
}
