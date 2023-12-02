package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;

@Config
@Autonomous
public class TiltTest extends LinearOpMode {
    public static double POS = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            Init.intakeTilt.setPosition(POS);
            tel.addData("seting pos to ", POS);
            tel.update();
        }
    }
}
