package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Init.METRES_TO_INCHES;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;

import java.util.ArrayList;

@Config
@Autonomous
public class AprilTagFollower extends LinearOpMode {
    private static final double CM_TO_INCH=0.394;
    public static double tagsize = 0.021; //todo for test purposes (my iphone's apriltag)
    public static double camx = -9*CM_TO_INCH, camy =6*CM_TO_INCH;
            //camera is 9cm to the left and 6cm to the front of the centre of robot
    @Override
    public void runOpMode() throws InterruptedException {
        double fx = 1430;
        double fy = 1430;
        double cx = 480;
        double cy = 620;
        AprilTagLibrary lib = AprilTagGameDatabase.getCenterStageTagLibrary();
        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        Init.init(hardwareMap);
        Init.camera.setPipeline(pipeline);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry tel = dashboard.getTelemetry();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.PI/2));
        waitForStart();
        boolean following = false;
        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
            tel.addData("size", detections.size());
            if (detections.size() > 0 && !following) {
                following = true;
                AprilTagPose pose = detections.get(0).pose;
                double x = pose.x*METRES_TO_INCHES-camx;

                tel.addData("x in", x);
                tel.addData("y in", pose.y* METRES_TO_INCHES);
                double z = pose.z*METRES_TO_INCHES-camy;
                tel.addData("z in", z);
                tel.update();
                Trajectory traj = drive.trajectoryBuilder(new Pose2d(0,0,Math.PI/2))
                        .lineTo(new Vector2d(x, z))
                        .build();
                drive.followTrajectory(traj);
                /*
                see https://github.com/AprilRobotics/apriltag#pose-estimation
                The coordinate system has the origin at the camera center.
                The z-axis points from the camera center out the camera lens.
                The x-axis is to the right in the image taken by the camera,
                and y is down.
                The tag's coordinate frame is centered at the center of the tag, with x-axis to the right, y-axis down, and z-axis into the tag.
                 */
            }
        }
    }
}
