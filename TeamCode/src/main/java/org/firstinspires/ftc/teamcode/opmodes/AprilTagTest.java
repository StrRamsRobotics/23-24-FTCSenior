package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous
public class AprilTagTest extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();

        waitForStart();
        while (opModeIsActive()) {
            Pose2d coords = Init.april.getCoords(2);
            tel.addData("x", coords.getX());
            tel.addData("y", coords.getY());
            tel.addData("heading", coords.getHeading());
            tel.update();
            Trajectory traj = Init.drive.trajectoryBuilder(Init.drive.getPoseEstimate())
                    .lineToLinearHeading(coords)
                    .build();
            Init.drive.followTrajectory(traj);
        }
    }
}
