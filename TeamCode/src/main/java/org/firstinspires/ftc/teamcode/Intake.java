package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class Intake {
    private Servo tilt;
    private CRServo intake;
    private final double TILT_UP = 0.5, TILT_DOWN = 0.1;
    public Intake(HardwareMap hardwareMap) {
        tilt = hardwareMap.get(Servo.class, "intakeTilt");
        intake = hardwareMap.get(CRServo.class, "intake");
    }
    public void run(boolean up) {
        Executors.newSingleThreadExecutor().execute(()->{
            intake.setPower(0.9);
            tilt.setPosition(up?TILT_UP:TILT_DOWN);
        });
        Executors.newSingleThreadScheduledExecutor().schedule(()->{
           intake.setPower(0);
        }, 2000, TimeUnit.MILLISECONDS);
    }
    public void release(boolean centre) throws InterruptedException {
        tilt.setPosition(0.85);
        Thread.sleep(400);
        intake.setPower(centre?0.65:0.85);
        Thread.sleep(centre?1200:800);
        intake.setPower(0);
    }

}
