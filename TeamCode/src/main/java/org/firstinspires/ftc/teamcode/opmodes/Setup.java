package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.Outtake;

@Config
@TeleOp
public class Setup extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        Init.intakeTilt.setPosition(0.45);
        Init.out.runSlide(180, 0.7);
        Init.rightOuttake.setPosition(0.47);
        Init.leftOuttake.setPosition(0.47);
        while (opModeIsActive()) {
        }
    }
}
