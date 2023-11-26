package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Init.CM_TO_INCH;
import static org.firstinspires.ftc.teamcode.Init.METRES_TO_INCH;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.PDController;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;


import java.util.ArrayList;

//apparently we're just doing this to face the board straightly (24 inches in front of board)
@Config
@Autonomous
public class BoardFollower extends LinearOpMode {
    //not supporting I
    public static PIDCoefficients RANGE_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients BEARING_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients YAW_PID = new PIDCoefficients(0, 0, 0);
    public static double RANGE_STOP = 12;
    //only support moving based on camera pos bc otherwise camera might not see the tag preemptively
    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        PDController rangeController = new PDController(RANGE_PID);
        PDController bearingController = new PDController(BEARING_PID);
        PDController yawController = new PDController(YAW_PID);
        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(2/METRES_TO_INCH, 822.317, 822.317, 319.495f, 242.502f);

        waitForStart();
        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
            //this is for blue
            AprilTagDetection centre = findTag(detections, 2), left = findTag(detections, 1), right = findTag(detections, 3);
            AprilTagDetection tag = centre != null ? centre : left != null ? left : right;
            if (tag==null) {
                tel.addData("No tag found", "stopping...");
                tel.update();
                Init.frontLeftDrive.setPower(0);
                Init.frontRightDrive.setPower(0);
                Init.backLeftDrive.setPower(0);
                Init.backRightDrive.setPower(0);
                continue;
            }
            AprilTagPoseFtc pose = poseToFtc(tag.pose);
            tel.addData("Id", tag.id);
            tel.addData("bearing", pose.bearing); //rotate CCW
            tel.addData("range", pose.range-RANGE_STOP); //move forward
            tel.addData("yaw", pose.yaw);//move right todo it might be move left?
            double bPower = bearingController.update(pose.bearing);
            double rPower = rangeController.update(pose.range-RANGE_STOP);
            double yPower = yawController.update(pose.yaw);
            tel.addData("bPower", bPower);
            tel.addData("rPower", rPower);
            tel.addData("yPower", yPower);
            Init.frontLeftDrive.setPower(yPower-bPower+rPower);
            Init.frontRightDrive.setPower(-yPower+bPower+rPower);
            Init.backLeftDrive.setPower(-yPower-bPower+rPower);
            Init.backRightDrive.setPower(yPower+bPower+rPower);
            tel.update();
        }
    }

    private AprilTagDetection findTag(ArrayList<AprilTagDetection> detections, int id) {
        for (AprilTagDetection detection : detections) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }
    
    private AprilTagPoseFtc poseToFtc(AprilTagPose detection) {
            double x =  detection.x;
            double y =  detection.z;
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
