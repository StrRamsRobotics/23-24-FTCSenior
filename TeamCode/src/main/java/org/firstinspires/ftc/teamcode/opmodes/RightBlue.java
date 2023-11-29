package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.Init.drive;
//thte one with purple pixel is off limits bc can't push purple pixe;
//should get 1 white pilxel after dropping purple pixel so 3 white pixels in total
@Autonomous
public class RightBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, -2.3, Math.PI));
        waitForStart();
        //do vision here
        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(25, -3, Math.PI/2)) //direction depends on vision
                .addDisplacementMarker(()->{
                    //release purple pixel
                })
                .lineToLinearHeading(new Pose2d(0, 0, Math.PI/2))
                .forward(48)
                .lineToConstantHeading(new Vector2d(24, 72))
                .addDisplacementMarker(()->{
                  //slides outake here
                })
                .lineToLinearHeading(new Pose2d(24, 48, -Math.PI/2))
                .forward(48)
                .lineToConstantHeading(new Vector2d(60, -24))
                .addDisplacementMarker(()->{
                    //take 2 white pixels
                })
                .lineToLinearHeading(new Pose2d(25, -3, Math.PI/2))
                .forward(72)
                .addDisplacementMarker(()->{
                    //slides outake here
                })
                .build();
        drive.followTrajectorySequence(traj);
    }
}
