package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Init;

public class Forward extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            Init.fr.setPower(0.5);
            Init.fl.setPower(0.5);
            Init.br.setPower(0.5);
            Init.bl.setPower(0.5);
        }
    }
}
