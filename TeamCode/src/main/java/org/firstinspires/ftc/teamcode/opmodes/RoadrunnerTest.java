package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name="rrtest")
public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Action action = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeTo(new Vector2d(0,72))
                .build();
        waitForStart();
        TelemetryPacket packet = new TelemetryPacket();
        action.run(packet);
        telemetry.addData("a", "ran"); telemetry.update();
        while (opModeIsActive()) {}

    }
}
