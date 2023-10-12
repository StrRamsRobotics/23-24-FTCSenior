package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous
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
        double tagsize = 0.127;
        AprilTagLibrary lib = AprilTagGameDatabase.getCenterStageTagLibrary();
        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        Init.init(hardwareMap);
        Init.camera.setPipeline(pipeline);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry tel = dashboard.getTelemetry();
        waitForStart();
        float curX = 0, curZ = 0;
        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
            tel.addData("size", detections.size());
            for (AprilTagDetection detection : detections) {
                AprilTagMetadata meta = lib.lookupTag(detection.id);
                float tagX = meta.fieldPosition.getData()[0];
                float tagZ = meta.fieldPosition.getData()[2];
                //note raw pose is in metres
                float x = (float) detection.pose.x*100;
                float y = (float) detection.pose.y*100; //todo is it bad that i just ignore y?
                float z = (float) detection.pose.z*100; //cm now
                Orientation orientation = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS);
                float yaw = orientation.thirdAngle;
                float rotX = (float) (x * Math.cos(yaw) - z * Math.sin(yaw));
                float rotZ = (float) (x * Math.sin(yaw) + z * Math.cos(yaw));
                curX = tagX - rotX;
                curZ = tagZ - rotZ;
                tel.addData("x", curX);
                tel.addData("z", curZ);
                tel.addData("raw x", x); tel.addData("raw y", y); tel.addData("raw z", z);
            }
            tel.update();
        }
        dashboard.stopCameraStream();
        Init.camera.closeCameraDeviceAsync(() -> {
        });
    }
}
