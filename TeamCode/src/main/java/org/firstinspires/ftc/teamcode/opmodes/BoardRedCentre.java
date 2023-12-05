package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Init.drive;
import static org.firstinspires.ftc.teamcode.Init.in;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.PropPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BoardRedCentre extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Init.init(hardwareMap);
        Init.prop = new PropPipeline(Init.Team.RED, Init.Side.EDGE);
        Init.camera.setPipeline(Init.prop);
        drive.setPoseEstimate(new Pose2d(0, -2.3, Math.PI));
        //do vision here
        // TrajectorySequence left1,left2,left3,centre1,centre2,centre3,right1,right2,right3;

//        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(left2.end())
//                .splineToConstantHeading(new Vector2d(56, 0), -Math.PI/2).build();
        tel.addData("done building trajs", "a"); tel.update();
        while (opModeInInit()) {
            tel.addData("prop", Init.prop.ans); tel.update();
        }
        waitForStart();
        Init.intakeTilt.setPosition(0.25);
        switch (Init.prop.ans) {
            case "left":
                Trajectory left1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                        .splineToLinearHeading(new Pose2d(26, -1.8, -Math.toRadians(65)), -Math.toRadians(65))
                        .build();
                drive.followTrajectory(left1);
                TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                        .splineToLinearHeading(new Pose2d(32.6, -40.3, -Math.PI/2), Math.PI)
                        .build();
                drive.followTrajectorySequence(left2);
                TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                        .lineToConstantHeading(new Vector2d(28.3, -36.3))
                        .splineToConstantHeading(new Vector2d(0, -35), Math.PI/2)
                        .build();
                in.release(false);
                drive.followTrajectorySequence(left3);
                break;
            case "centre":
                Trajectory centre1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                        .splineToLinearHeading(new Pose2d(22, 1, -Math.toRadians(165)), Math.toRadians(165))
                        .build();
                drive.followTrajectory(centre1);
                TrajectorySequence centre2 = drive.trajectorySequenceBuilder(centre1.end())
                        .splineToLinearHeading(new Pose2d(25, -40.8, -Math.PI/2), Math.PI)
                        .build();
                drive.followTrajectorySequence(centre2);
                TrajectorySequence centre3 = drive.trajectorySequenceBuilder(centre2.end())
                        .lineToConstantHeading(new Vector2d(24, -35))
                        .lineToConstantHeading(new Vector2d(0, -35))
                        .build();
                in.release(true);
                drive.followTrajectorySequence(centre3);
                break;
            case "right":
                Trajectory right1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                        .lineToLinearHeading(new Pose2d(20, -2.6, Math.PI))
                        .build();
                drive.followTrajectory(right1);
                TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                        .lineToLinearHeading(new Pose2d(19, -41.3, -Math.PI/2))
                        .build();
                drive.followTrajectorySequence(right2);
                TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
                        .lineToConstantHeading(new Vector2d(19, -37))
                        .splineToConstantHeading(new Vector2d(0, -35), Math.PI/2)
                        .build();
                in.release(false);
                drive.followTrajectorySequence(right3);
                break;
        }
        //Pose2d aprilCoords = Init.april.getCoords(2);
//        tel.addData("xapril", aprilCoords.getX());
//        tel.addData("yapril", aprilCoords.getY());
//        tel.update();}
    }
}