package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climb {
    private DcMotorEx leftClimb, rightClimb;
    public Climb(HardwareMap hardwareMap) {
        this.leftClimb = hardwareMap.get(DcMotorEx.class, "leftClimb");
        this.rightClimb = hardwareMap.get(DcMotorEx.class, "rightClimb");
    }
    public void runClimb(int position, double power) {
        leftClimb.setTargetPosition(position);
        rightClimb.setTargetPosition(position);
        leftClimb.setPower(power);
        rightClimb.setPower(power);
        leftClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
