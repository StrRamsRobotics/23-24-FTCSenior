package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;

@Config
@TeleOp
public class ServoJoystick extends LinearOpMode {
    public static double POS = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        Gamepad pad = this.gamepad1;
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        while (opModeIsActive()) {
            boolean a = pad.a||pad.cross;
            tel.addData("a", a);
            if (a) {
                servo.setPosition(POS);
            }
            tel.update();
         }
    }
}
