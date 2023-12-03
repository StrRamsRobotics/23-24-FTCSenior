package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    private static DcMotor leftSlide;
    private static DcMotor rightSlide;
    private static Servo leftOuttake, rightOuttake;
    public static final double EXTENSION_POS = 0.94, RETRACTION_POS = 0.43;

    public Outtake(HardwareMap hardwareMap) {
        this.leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        this.rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        this.leftOuttake = hardwareMap.get(Servo.class, "leftOuttake");
        this.rightOuttake = hardwareMap.get(Servo.class, "rightOuttake");
        this.rightOuttake.setDirection(Servo.Direction.REVERSE);
    }
    public void extendAndRetract(int position, double power, double runServoDistance) {
        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(position-position())>runServoDistance);
        leftOuttake.setPosition(EXTENSION_POS); //setposition is blocking
        leftOuttake.setPosition(RETRACTION_POS);
        runSlide(0, 0.5);
    }
    public void runSlide(int position, double power) {
        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runServo(boolean extend) {
        leftOuttake.setPosition(extend?EXTENSION_POS:RETRACTION_POS);
        rightOuttake.setPosition(extend?EXTENSION_POS:RETRACTION_POS);
    }
    public int position() {
        return (leftSlide.getCurrentPosition()+rightSlide.getCurrentPosition())/2;
    }
}