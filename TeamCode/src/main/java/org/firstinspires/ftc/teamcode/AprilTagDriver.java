package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.concurrent.Executors;

public class AprilTagDriver {
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline pipeline;

    public AprilTagDriver(OpenCvCamera camera) {
        this.camera = camera;
        pipeline = new AprilTagDetectionPipeline(0.0508, 947.118203072, 947.118203072, 357.858233883, 252.027176542);
    }

    public Pose2d getCoords(int id) {
        camera.setPipeline(pipeline);
        Executors.newSingleThreadExecutor().execute(() -> {
            AprilTagDetection cur = null;
            boolean flag = false;
            while (!flag) {
                ArrayList<AprilTagDetection> det = pipeline.getLatestDetections();
                for (AprilTagDetection d : det) {
                    if (d.id == id) {
                        flag = true;
                        cur = d;
                        break;
                    }
                }
            }
            AprilTagPoseFtc pose = poseToFtc(cur.pose);

        });
        return null;
    }
    private Pose2d darren(AprilTagPoseFtc pose) {
        double theta = pose.yaw;
        double alpha = Init.drive.getPoseEstimate().getHeading();
        double dx = pose.x, dy = pose.y;
        double x0 = 0, y0 = 0; //todo x0 y0 is offset from camera to centre of robot
        double h = 24;

        return null;
    }
    private AprilTagPoseFtc poseToFtc(AprilTagPose detection) {
        double x = detection.x;
        double y = detection.z;
        double z = -detection.y;

        Orientation rot = Orientation.getOrientation(detection.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS);
        double yaw = -rot.firstAngle;
        double roll = rot.thirdAngle;
        double pitch = rot.secondAngle;

        double range = Math.hypot(x, y);
        double bearing = AngleUnit.RADIANS.fromUnit(AngleUnit.RADIANS, Math.atan2(-x, y));
        double elevation = AngleUnit.RADIANS.fromUnit(AngleUnit.RADIANS, Math.atan2(z, y));
        return new AprilTagPoseFtc(x, y, z, yaw, roll, pitch, range, bearing, elevation);

    }
    }
