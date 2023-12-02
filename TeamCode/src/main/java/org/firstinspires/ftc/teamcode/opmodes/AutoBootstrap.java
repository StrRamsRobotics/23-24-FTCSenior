package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;

import static org.firstinspires.ftc.teamcode.Init.Team.*;
import static org.firstinspires.ftc.teamcode.Init.Side.*;

@Autonomous
public class AutoBootstrap extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        Gamepad pad = this.gamepad1;
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        Init.Team team=null;
        Init.Side side=null;
        while (opModeInInit()) {
            String text1="Press B for red, X for blue",text2="Press Y for left, A for right";
            if (pad.dpad_up &&team!=null&&side!=null) {
                tel.addData("Inited with", team +" "+ side);
                tel.update();
                break;
            }
            if (pad.b) {
                team = RED;
                text1 = "Red";
            }
            if (pad.x) {
                team = BLUE;
                text1 = "Blue";
            }
            if (pad.y) {
                side = BOARD;
                text2 = "Left";
            }
            if (pad.a) {
                side = EDGE;
                text2 = "Right";
            }
            tel.addData("Red or blue?", text1);
            tel.addData("Left or right?", text2);
            tel.addData("Press Dpad up to confirm choices", "");
            tel.update();
        }
        waitForStart();
        //call base auto func here
    }
}
