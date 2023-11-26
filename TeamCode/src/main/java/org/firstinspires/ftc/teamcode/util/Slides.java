package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Slides {
    private static DcMotor leftSlide;
    private static DcMotor rightSlide;

    public Slides(DcMotor leftSlide, DcMotor rightSlide) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
    }
    public void runToPosition(int position, double power) {
        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runToRelativePosition(int position, double power) {
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + position);
        rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + position);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public int position() {
        return (leftSlide.getCurrentPosition()+rightSlide.getCurrentPosition())/2;
    }
}
