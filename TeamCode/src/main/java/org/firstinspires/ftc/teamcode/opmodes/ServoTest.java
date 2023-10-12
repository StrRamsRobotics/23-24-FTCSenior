package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

//todo test this again
@Autonomous
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServoImplEx servo = hardwareMap.get(CRServoImplEx.class, "servo");
        servo.setPwmEnable();
        waitForStart();
        servo.setPower(1);
        while (opModeIsActive()) {
            servo.setPower(1);
            sleep(2000);
            servo.setPower(-1);
        }
    }
}
