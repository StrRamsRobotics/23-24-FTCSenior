package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.wrappers.Chassis;
import org.firstinspires.ftc.teamcode.wrappers.Game;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagAction extends AutoAction {
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static final double FX = 1042;
    public static final double FY = 781;
    public static final double CX = 800;
    public static final double CY = 600;
    public static final double DETECTION_X = 0.05;

    public static final double TAGSIZE = 0.166;

    public int numFramesWithoutDetection = 0;

    public static final float DECIMATION_HIGH = 3;
    public static final float DECIMATION_LOW = 2;
    public static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    public static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public static final int LEFT_BLUE_ID = 1;
    public static final int CENTER_BLUE_ID = 2;
    public static final int RIGHT_BLUE_ID = 3;
    public static final int LEFT_RED_ID = 4;
    public static final int CENTER_RED_ID = 5;
    public static final int RIGHT_RED_ID = 6;

    public int team = 0;
    public int route = 0;
    public int counter = 0;

    public AprilTagAction(Chassis chassis, int team, int route) {
        super(chassis);
        this.team = team;
        this.route = route;
    }

    public void tick() {
        if (!this.isInitialized) {
            int cameraMonitorViewId = chassis.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", chassis.hardwareMap.appContext.getPackageName());
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(TAGSIZE, FX, FY, CX, CY);

            chassis.camera.setPipeline(aprilTagDetectionPipeline);
            chassis.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    chassis.camera.startStreaming(AprilTagDetectionPipeline.IMAGE_WIDTH, AprilTagDetectionPipeline.IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
            this.isInitialized = true;
        }
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        counter++;
        chassis.logHelper.addData("Counter", counter);
        if(detections != null) {
            if(detections.size() == 0) {
                chassis.logHelper.addData("AprilTag Action", "No AprilTag Detected");
                numFramesWithoutDetection++;
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
            }
            else {
                numFramesWithoutDetection = 0;
                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }
                for(AprilTagDetection detection : detections) {
                    chassis.logHelper.addData("AprilTag ID", detection.id);
                    chassis.logHelper.addData("AprilTag X", detection.pose.x);
                    chassis.logHelper.addData("AprilTag Y", detection.pose.y);
                    chassis.logHelper.addData("AprilTag Z", detection.pose.z);
                    if (
                            detection.id == LEFT_BLUE_ID && route == 0 && team == Game.BLUE_TEAM ||
                            detection.id == CENTER_BLUE_ID && route == 1 && team == Game.BLUE_TEAM ||
                            detection.id == RIGHT_BLUE_ID && route == 2 && team == Game.BLUE_TEAM ||
                            detection.id == LEFT_RED_ID && route == 0 && team == Game.RED_TEAM ||
                            detection.id == CENTER_RED_ID && route == 1 && team == Game.RED_TEAM ||
                            detection.id == RIGHT_RED_ID && route == 2 && team == Game.RED_TEAM
                    ) {
                        if (detection.pose.x >= DETECTION_X) {
                            chassis.logHelper.addData("AprilTag Action", "Strafe Left");
                            chassis.fr.setPower(Chassis.MOVE_POWER);
                            chassis.fl.setPower(-Chassis.MOVE_POWER);
                            chassis.br.setPower(-Chassis.MOVE_POWER);
                            chassis.bl.setPower(Chassis.MOVE_POWER);
                        } else if (detection.pose.x <= -DETECTION_X) {
                            chassis.logHelper.addData("AprilTag Action", "Strafe Right");
                            chassis.fr.setPower(-Chassis.MOVE_POWER);
                            chassis.fl.setPower(Chassis.MOVE_POWER);
                            chassis.br.setPower(Chassis.MOVE_POWER);
                            chassis.bl.setPower(-Chassis.MOVE_POWER);
                        } else {
                            chassis.logHelper.addData("AprilTag Action", "AprilTag Centered");
                            chassis.fl.setPower(0);
                            chassis.fr.setPower(0);
                            chassis.bl.setPower(0);
                            chassis.br.setPower(0);
                            chassis.camera.closeCameraDeviceAsync(() -> {
                            });
                            active = false;
                        }
                    }
                }
            }
        }
        else {
            chassis.logHelper.addData("AprilTag Action", "No AprilTag Detected");
        }
    }
}