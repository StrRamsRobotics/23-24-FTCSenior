package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {
    public static double pos =0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        while (opModeIsActive()) {
            servo.setPosition(pos);
            tel.addData("seting pos to ", pos);
            tel.update();
        }

    }
}
