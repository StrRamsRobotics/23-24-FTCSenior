package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Init.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.Init.in;
import static org.firstinspires.ftc.teamcode.Init.rightClimb;

import java.lang.reflect.Field;
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
        Init.rightOuttake.setPosition(0.47);
        Init.leftOuttake.setPosition(0.47);
        TrajectorySequence boardTraj=null;
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
                boardTraj = left2;
                drive.followTrajectorySequence(left2);
                in.release(false);
//                Trajectory left4 = drive.trajectoryBuilder(left3.end())
//                        .splineToConstantHeading(new Vector2d(50, 35), Math.PI/2)
//                        .splineToLinearHeading(new Pose2d(16.5, 40.3, Math.PI/2), Math.PI)
//                        .build();
//                drive.followTrajectory(left4);
                break;
            case "centre":
                Trajectory centre1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                        .splineToLinearHeading(new Pose2d(25, -1, Math.toRadians(165)), Math.toRadians(-15))
                        .build();
                drive.followTrajectory(centre1);
                TrajectorySequence centre2 = drive.trajectorySequenceBuilder(centre1.end())
                        .splineToLinearHeading(new Pose2d(25, 40.7, Math.PI/2), Math.PI)
                        .build();
                boardTraj = centre2;
                drive.followTrajectorySequence(centre2);
//                Trajectory centre3 = drive.trajectoryBuilder(centre2.end(), true)
//                        .splineToConstantHeading(new Vector2d(0, 35), -Math.PI/2)
//                        .build();
//                in.release(true);
//                drive.followTrajectory(centre3);
                break;
            case "right":
                Trajectory right1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                        .splineToLinearHeading(new Pose2d(25, 4, Math.toRadians(65)), Math.toRadians(65))
                        .build();
                drive.followTrajectory(right1);
                TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                        .lineToLinearHeading(new Pose2d(28.3, 42.0, Math.PI/2))
                        .build();
                boardTraj = right2;
                drive.followTrajectorySequence(right2);
//                Trajectory right3 = drive.trajectoryBuilder(right2.end(), true)
//                        .splineToConstantHeading(new Vector2d(0, 35), -Math.PI/2)
//                        .build();
//                in.release(false);
//                drive.followTrajectory(right3);
                break;
        }
        Trajectory white1 = drive.trajectoryBuilder(boardTraj.end(), true)
                .splineToSplineHeading(new Pose2d(52, 12,-Math.PI/2+0.0001), -Math.PI/2)
                .splineToConstantHeading(new Vector2d(56, -66), -Math.PI/2)
                .build();

        TrajectoryFollower fol = drive.follower;
        try {
            Field aderr = TrajectoryFollower.class.getDeclaredField("admissibleError");
            Field tmo = TrajectoryFollower.class.getDeclaredField("timeout");
            tmo.setAccessible(true);
            aderr.setAccessible(true);
            tmo.set(fol, 2);
            aderr.set(fol, new Pose2d(0.01, 0.01, Math.toRadians(0.01)));
            telemetry.addData("timeout", tmo.get(fol));
            telemetry.update();
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }

        drive.followTrajectory(white1);
        try {
            Field aderr = TrajectoryFollower.class.getDeclaredField("admissibleError");
            Field tmo = TrajectoryFollower.class.getDeclaredField("timeout");
            tmo.setAccessible(true);
            aderr.setAccessible(true);
            aderr.set(fol, new Pose2d(0.5, 0.5, Math.toRadians(5.0)));
            tmo.set(fol, 0.5);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
/*        Init.rightOuttake.setPosition(0.47);
        Init.leftOuttake.setPosition(0.47);
        Init.in.run(0);
        Init.in.run(1);
        Init.in.intake.setPower(0.7);*/
        while (opModeIsActive()) {
        }
        //Pose2d aprilCoords = Init.april.getCoords(2);
//         telemetry.addData("xapril", aprilCoords.getX());
//         telemetry.addData("yapril", aprilCoords.getY());
//         telemetry.update();}
    }
}
