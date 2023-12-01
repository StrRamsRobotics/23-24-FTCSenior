package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Init.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import static org.firstinspires.ftc.teamcode.Init.out;
import static org.firstinspires.ftc.teamcode.Init.in;

//left means near the board
@Autonomous
public class LeftBlueEdge extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 2.3, Math.PI));
        waitForStart();
        //do vision here
        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(25, 4, Math.PI/2)) //direction depends on vision
                .addDisplacementMarker(()->{
                    in.release();
                })
                .lineToConstantHeading(new Vector2d(24, 60))
                .addDisplacementMarker(()->{

                    out.extendAndRetract(300, 0.5, 0);
                })
                .lineToLinearHeading(new Pose2d(0, 48, -Math.PI/2))
                .forward(48)
                .lineToConstantHeading(new Vector2d(60, -10))
                .addDisplacementMarker(()->{
                    in.run(true);
                })
                .lineToLinearHeading(new Pose2d(0, 0, Math.PI/2))
                .forward(48)
                .lineToConstantHeading(new Vector2d(24, 60))
                .addDisplacementMarker(()->{
                    out.extendAndRetract(300, 0.5, 0);
                })
                .build();
        drive.followTrajectorySequence(traj);
    }
}
