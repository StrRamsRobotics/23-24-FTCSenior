package com.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.concurrent.Executors;

public class MyClass {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.uiScale", "1.0");
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(new Pose2d(24, 48, Math.toRadians(90)))
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(1440), Math.toRadians(720), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(24, 48, -Math.PI/2))
                        .splineTo(new Vector2d(56, 35), 0)
                        .splineTo(new Vector2d(56, 4), -Math.PI/2)
                        .splineTo(new Vector2d(56, -60), -Math.PI/2)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .addEntity(myBot)
                .start();
    }
}