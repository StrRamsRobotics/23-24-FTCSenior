package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Init.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import static org.firstinspires.ftc.teamcode.Init.out;
import static org.firstinspires.ftc.teamcode.Init.in;

//left means near the board
@Autonomous
public class LeftBlueEdge extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        Init.init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 2.3, Math.PI));
        waitForStart();
        //do vision here
        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(24)
                .turn(Math.PI / 2)
                .addDisplacementMarker(() -> {
                    in.release();
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(0, 0))
                .lineToLinearHeading(new Pose2d(0, 24, Math.PI / 2))
                .lineToConstantHeading(new Vector2d(24, 24))
                .build();
        drive.followTrajectorySequence(traj);
        //Pose2d aprilCoords = Init.april.getCoords(2);
//        tel.addData("xapril", aprilCoords.getX());
//        tel.addData("yapril", aprilCoords.getY());
//        tel.update();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end()) //takes .5 secs
               // .lineToLinearHeading(aprilCoords)
                .addDisplacementMarker(() -> {
                    //uses claw
                })
                .waitSeconds(5)
                .lineToLinearHeading(new Pose2d(0, 0, -Math.PI / 2))
                .forward(48)
                .lineToConstantHeading(new Vector2d(60, -50))
                .addDisplacementMarker(() -> {
                    in.run(true);
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(0, -48, Math.PI / 2))
                .forward(48)
                .lineToConstantHeading(new Vector2d(24, 24))
                .addDisplacementMarker(() -> {
                    out.extendAndRetract(300, 0.5, 0);
                })
                .waitSeconds(5)
                .build();
        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(traj);
    }
}
