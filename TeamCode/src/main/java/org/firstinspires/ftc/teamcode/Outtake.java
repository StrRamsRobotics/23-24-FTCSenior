package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    private static DcMotor leftSlide;
    private static DcMotor rightSlide;
    private static Servo outtake;
    public static double EXTENSION_POS = 0.94, RETRACTION_POS = 0.43;

    public Outtake(HardwareMap hardwareMap) {
        this.leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        this.rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        this.outtake = hardwareMap.get(Servo.class, "outtake");
    }
    public void extendAndRetract(int position, double power, double runServoDistance) {
        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(position-position())>runServoDistance);
        outtake.setPosition(EXTENSION_POS); //setposition is blocking
        outtake.setPosition(RETRACTION_POS);
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
    public void runServo(double position, double power) {
        outtake.setPosition(position);
    }
    public int position() {
        return (leftSlide.getCurrentPosition()+rightSlide.getCurrentPosition())/2;
    }
}