package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

//todo only usable once we get apriltags installed on field
//apparently we're just doing this to face the board straightly (24 inches in front of board)
@Autonomous
public class BoardFollower extends LinearOpMode {
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
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            AprilTagPoseFtc target = null;
            TelemetryPacket packet = new TelemetryPacket();
            ArrayList<AprilTagDetection> detections= aprilProcessor.getDetections();
            for (AprilTagDetection detection : detections) {
                if (!detection.metadata.name.contains("AllianceCenter")) {
                    continue;
                }
                if (target==null) {
                    target = detection.ftcPose;
                }
                tel.addData("bearing", Math.toDegrees(detection.ftcPose.bearing));
            }
            Pose2d cur = drive.getPoseEstimate();
            packet.fieldOverlay().setFill("blue").fillCircle(cur.getX(), cur.getY(), 25);

            if (target==null) {
                continue;
            }
            double x = cur.getX() - target.x;
            double y = cur.getY() - (target.y - Math.signum(target.y) * 24);
            packet.fieldOverlay().setFill("green").fillCircle(x, y, 25);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            Trajectory traj = drive.trajectoryBuilder(cur)
                    .lineToLinearHeading(new Pose2d(x, y, target.bearing))
                    .build();
            drive.followTrajectory(traj);
        }
    }
}
