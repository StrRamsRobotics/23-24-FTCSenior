package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous
@Disabled //bc don't wanna refactor this to visionportal
public class AprilTagTest extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        // Lens intrinsics
        // UNITS ARE PIXELS
//i believe the pose is from camera lens' point of view

        //assume we're using c270 logitech bc i think there's only 1 logitech 720p cam
        double fx = 1430;
        double fy = 1430;
        double cx = 480;
        double cy = 620;
        // 5 inches (metres)
        double tagsize = 0.05;
        AprilTagLibrary lib = AprilTagGameDatabase.getCenterStageTagLibrary();
        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        Init.init(hardwareMap);
        //Init.camera.setPipeline(pipeline);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry tel = dashboard.getTelemetry();
        waitForStart();
        double curX = 0, curY = 0;
        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
            tel.addData("size", detections.size());
            for (AprilTagDetection detection : detections) {
                AprilTagMetadata meta = lib.lookupTag(detection.id);
                float[] fieldPos = meta.fieldPosition.getData();
                double fieldRot = meta.fieldOrientation.z; //degrees CCW from ref (red alliance station) eg. mosaic apriltags are 90 degrees
                double fieldX = fieldPos[0];
                double fieldY = fieldPos[1];
                double relRot = Math.atan2(-detection.pose.x, detection.pose.z); //negative x bc we wanna turn CCW to get there
                double curRot = fieldRot-relRot; //todo this calc could be wrong
                double rotX = Math.cos(curRot)*detection.pose.x - Math.sin(curRot)*detection.pose.z;
                double rotY = Math.sin(curRot)*detection.pose.x + Math.cos(curRot)*detection.pose.z;
                curX = meta.fieldPosition.get(0)-rotX; //z is up in this case
                curY = meta.fieldPosition.get(1)-rotY;
                tel.addData("x", curX);
                tel.addData("z", curY);
                tel.addData("raw x", fieldX); tel.addData("raw z", fieldY);
            }
            tel.update();
        }
    }
}
