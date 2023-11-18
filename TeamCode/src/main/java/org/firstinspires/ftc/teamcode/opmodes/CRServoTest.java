package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous
public class CRServoTest extends LinearOpMode {
    public static double power = 0.9;//note that 1 doen't work (there's a max power)
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        CRServoImplEx servo = hardwareMap.get(CRServoImplEx.class, "servo");
        waitForStart();
        while (opModeIsActive()) {
            servo.setPower(power);
            tel.addData("p", servo.getPower()); tel.update();
        }
    }
}
