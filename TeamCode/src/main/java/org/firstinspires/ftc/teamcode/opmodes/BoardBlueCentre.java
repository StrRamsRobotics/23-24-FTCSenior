package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Init.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.PropPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.Init.in;
import static org.firstinspires.ftc.teamcode.Init.rightClimb;

import java.util.concurrent.Executors;

//left means near the board
@Autonomous
public class BoardBlueCentre extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Init.init(hardwareMap);
        Init.prop = new PropPipeline(Init.Team.BLUE, Init.Side.BOARD);
        Init.camera.setPipeline(Init.prop);
        drive.setPoseEstimate(new Pose2d(0, 2.3, Math.PI));
        //do vision here
//        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(left2.end())
//                .splineToConstantHeading(new Vector2d(56, 0), -Math.PI/2).build();
         telemetry.addData("done building trajs", "a");  telemetry.update();
        while (opModeInInit()) {
             telemetry.addData("prop", Init.prop.ans);  telemetry.update();
        }

        waitForStart();

        Init.intakeTilt.setPosition(0.25);
//        switch (Init.prop.ans) {
        switch ("left") {
            case "left":
                TrajectorySequence left1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(19, 4.5, Math.PI))
                        .build();
                drive.followTrajectorySequence(left1);
                TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                        .splineToLinearHeading(new Pose2d(16.5, 40.3, Math.PI/2), Math.PI)
                        .build();
                drive.followTrajectorySequence(left2);
                Init.intakeTilt.setPosition(0.65);
                Trajectory left3 = drive.trajectoryBuilder(left2.end(), true)
                        .splineToConstantHeading(new Vector2d(56, 35), -Math.PI/2)
                        .splineToSplineHeading(new Pose2d(56, 4, -Math.PI/2), -Math.PI/2)
                        .splineTo(new Vector2d(56, -60), -Math.PI/2)
                        .build();
                in.release(false);
                drive.followTrajectory(left3);
                Trajectory left4 = drive.trajectoryBuilder(left3.end())
                        .splineToConstantHeading(new Vector2d(50, 35), Math.PI/2)
                        .splineToLinearHeading(new Pose2d(24, 48, Math.PI/2), Math.PI)
                        .build();
                drive.followTrajectory(left4);
                break;
            case "centre":
                Trajectory centre1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                        .splineToLinearHeading(new Pose2d(25, -1, Math.toRadians(165)), Math.toRadians(-15))
                        .build();
                drive.followTrajectory(centre1);
                TrajectorySequence centre2 = drive.trajectorySequenceBuilder(centre1.end())
                        .splineToLinearHeading(new Pose2d(25, 40.7, Math.PI/2), Math.PI)
                        .build();
                drive.followTrajectorySequence(centre2);
                Trajectory centre3 = drive.trajectoryBuilder(centre2.end(), true)
                        .splineToConstantHeading(new Vector2d(0, 35), -Math.PI/2)
                        .build();
                in.release(true);
                drive.followTrajectory(centre3);
                break;
            case "right":
                Trajectory right1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                        .splineToLinearHeading(new Pose2d(25, 4, Math.toRadians(65)), Math.toRadians(65))
                        .build();
                drive.followTrajectory(right1);
                TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                        .lineToLinearHeading(new Pose2d(28.3, 42.0, Math.PI/2))
                        .build();
                drive.followTrajectorySequence(right2);
                Trajectory right3 = drive.trajectoryBuilder(right2.end(), true)
                        .splineToConstantHeading(new Vector2d(0, 35), -Math.PI/2)
                        .build();
                in.release(false);
                drive.followTrajectory(right3);
                break;
        }
        //Pose2d aprilCoords = Init.april.getCoords(2);
//         telemetry.addData("xapril", aprilCoords.getX());
//         telemetry.addData("yapril", aprilCoords.getY());
//         telemetry.update();}
    }
}
