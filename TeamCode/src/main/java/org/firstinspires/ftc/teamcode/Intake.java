package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class Intake {
    public Servo tilt;
    public CRServo intake,outtakeWheel;
    private double[] heights = {0.66, 0.67}; //2 was previously 0.68
    private final double TILT_FIRST = 0.66, TILT_DOWN = 0.68;

    public Intake(HardwareMap hardwareMap) {
        outtakeWheel = hardwareMap.get(CRServo.class, "outtakeWheel");
        tilt = hardwareMap.get(Servo.class, "intakeTilt");
        intake = hardwareMap.get(CRServo.class, "intake");
    }
    public void run(int idx) throws InterruptedException {
            tilt.setPosition(heights[idx]);
            Thread.sleep(250);
        intake.setPower(0.7);
        Thread.sleep(1000);
           intake.setPower(0);
           outtakeWheel.setPower(0);
    }
    public void release(boolean centre) throws InterruptedException {
        tilt.setPosition(0.75);
        Thread.sleep(1000);
        intake.setPower(centre?-0.75:-0.85);
        Thread.sleep(centre?1200:800);
        tilt.setPosition(0.24);
        intake.setPower(0);
    }

}
