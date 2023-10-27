package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;

@TeleOp
public class Joystick extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        waitForStart();
        Gamepad pad = this.gamepad1;
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        while (opModeIsActive()) {
            float lx = pad.left_stick_x;
            float ly = -pad.left_stick_y;
            float rx = pad.right_stick_x;
            tel.addData("raw lx", lx);
            tel.addData("raw ly", ly);
            tel.addData("raw rx", rx);
            tel.update();
            if (Math.abs(lx)<0.1) lx = 0; if (Math.abs(ly)<0.1) ly=0; if (Math.abs(rx)<0.1) rx = 0;
            lx=smooth(lx);ly=smooth(ly); rx = smooth(rx); //rx+ry reverses when plugged in laptop
            Init.fl.setPower(Math.min(1, lx + ly+rx));
            Init.fr.setPower(Math.min(1, -lx + ly-rx));
            Init.bl.setPower(Math.min(1, -lx + ly+rx));
            Init.br.setPower(Math.min(1, lx + ly-rx));
         }
    }
    private float smooth(float in) {
        return in*in*in; //more non-linear than 2^x
    }
}
