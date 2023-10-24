package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

//todo only usable once we get apriltags installed on field
@Autonomous
public class FtcAprilTagTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //no init for this bc diff camera init
        CameraStreamProcessor streamer = new CameraStreamProcessor();
        AprilTagProcessor aprilProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(1430, 1430, 480, 620)
                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(aprilProcessor)
                .addProcessor(streamer)
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .setCameraResolution(new Size(320, 240))
                .build();
        FtcDashboard.getInstance().startCameraStream(streamer, 30);
        double curX = 0, curY = 0;
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections= aprilProcessor.getDetections();
            for (AprilTagDetection detection : detections) {
                tel.addData("bearing", Math.toDegrees(detection.ftcPose.bearing));
                double heading = detection.metadata.fieldOrientation.z + Math.PI*2-detection.ftcPose.yaw; //this should make sense
                //fieldorientation is degrees CCW from ref (red alliance station) eg. mosaic apriltags are 90 degrees
                double rotX = Math.cos(heading) * detection.ftcPose.x - Math.sin(heading) * detection.ftcPose.y;
                double rotY = Math.sin(heading) * detection.ftcPose.x + Math.cos(heading) * detection.ftcPose.y;

                curX = detection.metadata.fieldPosition.get(0)-rotX;
                curY = detection.metadata.fieldPosition.get(1)-rotY;
                tel.addData("x", curX);
                tel.addData("y", curY);
                tel.addData("heading", Math.toDegrees(heading));
                tel.update();
            }
        }
    }
}
