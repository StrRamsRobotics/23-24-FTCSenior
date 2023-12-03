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
import org.firstinspires.ftc.teamcode.PropPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.Init.in;
import static org.firstinspires.ftc.teamcode.Init.rightClimb;

import java.util.Vector;

//left means near the board
@Autonomous
public class FarBlueCentre extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Init.init(hardwareMap);
        Init.prop = new PropPipeline(Init.Team.BLUE, Init.Side.EDGE);
        Init.camera.setPipeline(Init.prop);
        drive.setPoseEstimate(new Pose2d(0, -2.3, Math.PI));
        //do vision here
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(19, -1.5, Math.PI))
                .lineToConstantHeading(new Vector2d(17, -1.5))
                .lineToConstantHeading(new Vector2d(17, -20.3))
                .lineToConstantHeading(new Vector2d(49.4, -20.3))
                .lineToConstantHeading(new Vector2d(49.4, 53))
                .build();
        Trajectory left1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToLinearHeading(new Pose2d(27, 4, Math.toRadians(-45)), Math.toRadians(-45))
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .lineToLinearHeading(new Pose2d(23, -8, Math.PI))
                .build();
        Trajectory left3 = drive.trajectoryBuilder(left2.end(), true)
                .splineTo(new Vector2d(49.4, 53), Math.PI)
                .build();
//        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
//                .splineToLinearHeading(new Pose2d(15, 40.3, Math.PI/2), Math.PI)
//                .build();
//        Trajectory left3 = drive.trajectoryBuilder(left2.end(), true)
//                .splineToConstantHeading(new Vector2d(0, 35), -Math.PI/2)
//                .build();
        Trajectory centre1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToLinearHeading(new Pose2d(20, 1, Math.toRadians(195)), Math.toRadians(195))
                .build();
        Init.intakeTilt.setPosition(0.1);
        TrajectorySequence centre2 = drive.trajectorySequenceBuilder(centre1.end())
                .splineToLinearHeading(new Pose2d(24, -19, Math.PI), Math.PI)
                .build();
        Trajectory centre3 = drive.trajectoryBuilder(centre2.end(), true)
                .splineTo(new Vector2d(49, 0), Math.PI/2)
                .splineTo(new Vector2d(49, 51), Math.PI/2)
                .build();
//        Trajectory right1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
//                .splineToLinearHeading(new Pose2d(25, -4, Math.toRadians(65)), Math.toRadians(65))
//                .build();
//        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
//                .lineToLinearHeading(new Pose2d(28.3, 41.3, Math.PI/2))
//                .build();
//        Trajectory right3 = drive.trajectoryBuilder(right2.end(), true)
//                .splineToConstantHeading(new Vector2d(0, 35), -Math.PI/2)
//                .build();
//        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(left2.end())
//                .splineToConstantHeading(new Vector2d(56, 0), -Math.PI/2).build();
        tel.addData("done building trajs", "a"); tel.update();
        waitForStart();
        Init.intakeTilt.setPosition(0.1);
        switch (Init.prop.ans) {
            case "left":
                drive.followTrajectory(left1);
//                drive.followTrajectorySequence(left2);
//                drive.followTrajectory(left3);
                break;
            case "centre":
                drive.followTrajectory(centre1);
                drive.followTrajectorySequence(centre2);
                drive.followTrajectory(centre3);
                break;
            case "right":
                drive.followTrajectorySequence(right1);
//                drive.followTrajectory(right2);
//                in.release(false);
//                drive.followTrajectory(right3);
                break;
        }
        tel.addData("prop", Init.prop.ans); tel.update();
        //Pose2d aprilCoords = Init.april.getCoords(2);
//        tel.addData("xapril", aprilCoords.getX());
//        tel.addData("yapril", aprilCoords.getY());
//        tel.update();}
    }
}
