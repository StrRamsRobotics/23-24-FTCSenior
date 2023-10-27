package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous
public class VoltageChecker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Voltage", voltageSensor.getVoltage());
            telemetry.update();
        }
    }
}
