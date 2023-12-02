package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Init.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import static org.firstinspires.ftc.teamcode.Init.out;
import static org.firstinspires.ftc.teamcode.Init.in;

import java.util.concurrent.Executors;

//left means near the board
@Autonomous
public class LeftBlueEdge extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
       // Telemetry tel = FtcDashboard.getInstance().getTelemetry();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Init.init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 2.3, Math.PI));
        //do vision here
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(19, 4.5, Math.PI))
//                .splineToConstantHeading(new Vector2d(12, 12), Math.PI)
                .build();

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(29, 41.3, Math.PI/2), Math.PI)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj11.end())
                .splineToConstantHeading(new Vector2d(56, 0), -Math.PI/2).build();
        waitForStart();
        //Pose2d aprilCoords = Init.april.getCoords(2);
//        tel.addData("xapril", aprilCoords.getX());
//        tel.addData("yapril", aprilCoords.getY());
//        tel.update();

        drive.followTrajectorySequence(traj1);
        Executors.newSingleThreadExecutor().execute(()->{
            Init.intakeTilt.setPosition(0.7);
        });
        drive.followTrajectorySequence(traj11);
        in.release();
        Init.leftClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Init.rightClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Init.rightClimb.setTargetPosition(3150);
        Init.leftClimb.setTargetPosition(3150);
        Init.rightClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Init.leftClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Init.leftClimb.setPower(0.5);
        Init.rightClimb.setPower(0.5);
        drive.followTrajectorySequence(traj2);
    }
}
