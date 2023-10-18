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
            lx=smooth(lx);ly=smooth(ly); rx = smooth(rx); //todo why does rotating work on ry instead of rx actually it's bc it reverses when plugged in laptop
            Init.fl.setPower(lx + ly+rx);
            Init.fr.setPower(-lx + ly-rx);
            Init.bl.setPower(-lx + ly+rx);
            Init.br.setPower(lx + ly-rx);
         }
    }
    private float smooth(float in) {
        return (float) (Math.signum(in)*(Math.pow(2, Math.abs(in))-1));
    }
}
