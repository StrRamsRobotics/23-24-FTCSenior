package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class VoltageChecker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        FtcDashboard.start(null);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            tel.addData("Voltage", voltageSensor.getVoltage());
            tel.update();
        }
    }
}
