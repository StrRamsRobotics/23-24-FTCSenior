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
        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(48, 0, Math.PI))
                .build();
        drive.followTrajectorySequence(test);
        while (opModeIsActive()) {}

    }
}
