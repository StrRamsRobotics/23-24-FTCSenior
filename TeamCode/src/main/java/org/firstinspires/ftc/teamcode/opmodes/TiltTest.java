package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;

@Config
@Autonomous
public class TiltTest extends LinearOpMode {
    public static double POS = 0;
    public static double POWER=0;
    @Override
    public void runOpMode() throws InterruptedException {
        //Init.init(hardwareMap);
        Servo outtake = hardwareMap.get(Servo.class, "intakeTilt");
        CRServo spin = hardwareMap.get(CRServo.class, "intake");
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            outtake.setPosition(POS);
            spin.setPower(POWER);
            tel.addData("seting pos to ", POS);
            tel.addData("setting power to ", POWER);
            tel.update();
        }
    }
}
