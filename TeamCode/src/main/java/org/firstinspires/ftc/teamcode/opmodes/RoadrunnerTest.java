package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Init.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        waitForStart();
        Init.rightOuttake.setPosition(0.47);
        Init.leftOuttake.setPosition(0.47);
        Init.intakeTilt.setPosition(0.3);
        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(48, 48, Math.PI), 0)
                .build();
        drive.followTrajectorySequence(test);
        TrajectorySequence test2=drive.trajectorySequenceBuilder(test.end())
                .splineToSplineHeading(new Pose2d(0,0,0), 0)
                .build();
        drive.followTrajectorySequence(test2);
        while (opModeIsActive()) {}

    }
}
