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

import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class BoardRedCentre extends LinearOpMode {
    private volatile boolean doneTraj = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Init.init(hardwareMap);
        Init.prop = new PropPipeline(Init.Team.RED, Init.Side.EDGE);
//        Init.camera.setPipeline(Init.prop);
        drive.setPoseEstimate(new Pose2d(0, -2.3, Math.PI));
        //do vision here
        AtomicReference<TrajectorySequence> left1 = new AtomicReference<>();
        AtomicReference<TrajectorySequence> left2 = new AtomicReference<>();
        AtomicReference<Trajectory> left3= new AtomicReference<>();
        AtomicReference<Trajectory> centre1= new AtomicReference<>();
        AtomicReference<TrajectorySequence> centre2= new AtomicReference<>();
        AtomicReference<TrajectorySequence> centre3= new AtomicReference<>();
        AtomicReference<Trajectory> right1= new AtomicReference<>();
        AtomicReference<TrajectorySequence> right2= new AtomicReference<>();
        AtomicReference<Trajectory> right3= new AtomicReference<>();
        Executors.newSingleThreadExecutor().execute(()->{
            left1.set(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(25, -0.6, -Math.toRadians(65)), -Math.toRadians(65))
                    .build());
            left2.set(drive.trajectorySequenceBuilder(left1.get().end())
                    .splineToLinearHeading(new Pose2d(28.3, -40.3, -Math.PI / 2), Math.PI)
                    .build());
            left3.set(drive.trajectoryBuilder(left2.get().end(), false)
                    .splineToConstantHeading(new Vector2d(0, -35), Math.PI / 2)
                    .build());
            centre1.set(drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                    .splineToLinearHeading(new Pose2d(20, 1, -Math.toRadians(165)), -Math.toRadians(165))
                    .build());
            centre2.set(drive.trajectorySequenceBuilder(centre1.get().end())
                    .splineToLinearHeading(new Pose2d(24, -40.3, -Math.PI / 2), Math.PI)
                    .build());
            centre3.set(drive.trajectorySequenceBuilder(centre2.get().end())
                    .lineToConstantHeading(new Vector2d(24, -35))
                    .lineToConstantHeading(new Vector2d(0, -35))
                    .build());
            right1.set(drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                    .lineToLinearHeading(new Pose2d(20, -0.1, Math.PI))
                    .build());
            right2.set(drive.trajectorySequenceBuilder(right1.get().end())
                    .lineToLinearHeading(new Pose2d(1, -41.3, -Math.PI / 2))
                    .build());
            right3.set(drive.trajectoryBuilder(right2.get().end(), false)
                    .splineToConstantHeading(new Vector2d(0, -35), Math.PI / 2)
                    .build());
            doneTraj = true;
        });
        
//        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(left2.end())
//                .splineToConstantHeading(new Vector2d(56, 0), -Math.PI/2).build();
        while (!isStarted()) {
            if (doneTraj) {
                tel.addData("done building trajs", "a"); tel.update();
            }
            tel.addData("prop", Init.prop.ans); tel.update();
            Thread.sleep(30);
        }
        waitForStart();
        Init.intakeTilt.setPosition(0.2);
        switch (Init.prop.ans) {
            case "left":
                drive.followTrajectorySequence(left1.get());
                drive.followTrajectorySequence(left2.get());
                in.release(false);
                drive.followTrajectory(left3.get());
                break;
            case "centre":
                drive.followTrajectory(centre1.get());
                drive.followTrajectorySequence(centre2.get());
                in.release(true);
                drive.followTrajectorySequence(centre3.get());
                break;
            case "right":
                drive.followTrajectory(right1.get());
                drive.followTrajectorySequence(right2.get());
                in.release(false);
                drive.followTrajectory(right3.get());
                break;
        }
        //Pose2d aprilCoords = Init.april.getCoords(2);
//        tel.addData("xapril", aprilCoords.getX());
//        tel.addData("yapril", aprilCoords.getY());
//        tel.update();}
    }
}