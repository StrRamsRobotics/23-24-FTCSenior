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
    private volatile boolean doneTraj = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Init.init(hardwareMap);
        Init.prop = new PropPipeline(Init.Team.RED, Init.Side.EDGE);
        drive.setPoseEstimate(new Pose2d(0, -2.3, Math.PI));
        //do vision here
       // TrajectorySequence left1,left2,left3,centre1,centre2,centre3,right1,right2,right3;

        TrajectorySequence left1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(25, -0.6, -Math.toRadians(65)), -Math.toRadians(65))
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .splineToLinearHeading(new Pose2d(28.3, -40.3, -Math.PI/2), Math.PI)
                .build();
        Trajectory left3 = drive.trajectoryBuilder(left2.end(), false)
                .splineToConstantHeading(new Vector2d(0, -35), Math.PI/2)
                .build();
        Trajectory centre1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToLinearHeading(new Pose2d(20, 1, -Math.toRadians(165)), -Math.toRadians(165))
                .build();
        TrajectorySequence centre2 = drive.trajectorySequenceBuilder(centre1.end())
                .splineToLinearHeading(new Pose2d(24, -40.3, -Math.PI/2), Math.PI)
                .build();
        TrajectorySequence centre3 = drive.trajectorySequenceBuilder(centre2.end())
                .lineToConstantHeading(new Vector2d(24, -35))
                .lineToConstantHeading(new Vector2d(0, -35))
                .build();
        Trajectory right1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .lineToLinearHeading(new Pose2d(20, -0.1, Math.PI))
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .lineToLinearHeading(new Pose2d(1, -41.3, -Math.PI/2))
                .build();
        Trajectory right3 = drive.trajectoryBuilder(right2.end(), false)
                .splineToConstantHeading(new Vector2d(0, -35), Math.PI/2)
                .build();
//        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(left2.end())
//                .splineToConstantHeading(new Vector2d(56, 0), -Math.PI/2).build();
        tel.addData("done building trajs", "a"); tel.update();
        while (opModeInInit()) {
            tel.addData("prop", Init.prop.ans); tel.update();
        }
        waitForStart();
        Init.intakeTilt.setPosition(0.2);
        switch (Init.prop.ans) {
            case "left":
                drive.followTrajectorySequence(left1);
                drive.followTrajectorySequence(left2);
                in.release(false);
                drive.followTrajectory(left3);
                break;
            case "centre":
                drive.followTrajectory(centre1);
                drive.followTrajectorySequence(centre2);
                in.release(true);
                drive.followTrajectorySequence(centre3);
                break;
            case "right":
                drive.followTrajectory(right1);
                drive.followTrajectorySequence(right2);
                in.release(false);
                drive.followTrajectory(right3);
                break;
        }
        //Pose2d aprilCoords = Init.april.getCoords(2);
//        tel.addData("xapril", aprilCoords.getX());
//        tel.addData("yapril", aprilCoords.getY());
//        tel.update();}
    }
}