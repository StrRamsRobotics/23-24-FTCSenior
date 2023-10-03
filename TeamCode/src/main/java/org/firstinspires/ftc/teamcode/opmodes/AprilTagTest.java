package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Init;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "AprilTagTest")
public class AprilTagTest extends LinearOpMode {
    //todo use enums for tag pos
    static final double FEET_PER_METER = 3.28084;
    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
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
        double tagsize = 0.166;
        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        Init.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry tel = dashboard.getTelemetry();
        waitForStart();
        float curX = 0, curY = 0;
        float tagX = 0, tagY = 0;
        /*while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
            tel.addData("size", detections.size());
            for (AprilTagDetection detection : detections) {
//                VectorF rotated = detection.pose.R.transform(new VectorF((float) detection.pose.x, (float) detection.pose.y, (float) detection.pose.z));
//                curX = tagX - rotated.get(0);
//                curY = tagY - rotated.get(2);
//                tel.addData("x", curX);
//                tel.addData("y", curY);
*//*                Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                tel.addLine(String.format("\nDetected tag ID=%d", detection.id));
                tel.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                tel.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                tel.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                tel.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                tel.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                tel.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
                tel.update();*//*
            }
            tel.update();
        }*/
        while (opModeIsActive())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if(detections != null)
            {
                tel.addData("FPS", Init.camera.getFps());
                tel.addData("Overhead ms", Init.camera.getOverheadTimeMs());
                tel.addData("Pipeline ms", Init.camera.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        pipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        pipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                        tel.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        tel.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        tel.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        tel.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        tel.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                        tel.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                        tel.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
                    }
                }

                tel.update();
            }

            sleep(20);
        }
        dashboard.stopCameraStream();
        Init.camera.closeCameraDeviceAsync(() -> {
        });
    }
}
