package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;

@Autonomous(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry tel = dashboard.getTelemetry();
        Init.init(hardwareMap);
        DcMotor dead = hardwareMap.get(DcMotor.class, "dead");
        waitForStart();
        dead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tel.addData("started", "a"); tel.update();
        while (opModeIsActive()) {
            tel.addData("Encoder", dead.getCurrentPosition());
            tel.update();
        }
    }
}
