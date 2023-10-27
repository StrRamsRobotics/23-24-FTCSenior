package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Init.CM_TO_INCH;
import static org.firstinspires.ftc.teamcode.Init.aprilTagProcessor;

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.PDController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

//apparently we're just doing this to face the board straightly (24 inches in front of board)
@Config
@Autonomous
public class BoardFollower extends LinearOpMode {
    //not supporting I
    public static PIDCoefficients RANGE_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients BEARING_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients YAW_PID = new PIDCoefficients(0, 0, 0);
    public static double RANGE_STOP = 24;
    //offsets from centre of robot to camera todo maybe try without camx and camy first
    public static double CAM_X = -9*CM_TO_INCH, CAM_Y =6*CM_TO_INCH;
    @Override
    public void runOpMode() throws InterruptedException {
        Init.init(hardwareMap);
        Telemetry tel = FtcDashboard.getInstance().getTelemetry();
        PDController rangeController = new PDController(RANGE_PID);
        PDController bearingController = new PDController(BEARING_PID);
        PDController yawController = new PDController(YAW_PID);
        waitForStart();
        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.id == 2) { //2 for blue, 5 for red
                    tel.addData("bearing", Math.toDegrees(detection.ftcPose.bearing)); //rotate CCW
                    tel.addData("range", detection.ftcPose.range-RANGE_STOP); //move forward
                    tel.addData("yaw", detection.ftcPose.yaw);//move right todo it might be move left?
                    double bPower = bearingController.update(detection.ftcPose.bearing);
                    double rPower = rangeController.update(detection.ftcPose.range-RANGE_STOP+CAM_Y);
                    double yPower = yawController.update(detection.ftcPose.yaw+CAM_X);
                    tel.addData("bPower", bPower);
                    tel.addData("rPower", rPower);
                    tel.addData("yPower", yPower);
                    Init.fl.setPower(yPower-bPower+rPower);
                    Init.fr.setPower(-yPower+bPower+rPower);
                    Init.bl.setPower(-yPower-bPower+rPower);
                    Init.br.setPower(yPower+bPower+rPower);
                    tel.update();
                    break;
                }
            }
        }
    }
}
