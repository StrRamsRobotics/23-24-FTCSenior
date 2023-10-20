package org.firstinspires.ftc.teamcode.opmodes.auto.deprecated;

import org.firstinspires.ftc.teamcode.opmodes.auto.pipelines.VisionPipeline;
import org.firstinspires.ftc.teamcode.wrappers.Game;
import org.firstinspires.ftc.teamcode.wrappers.deprecated.ChassisOld;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class MainAutoOld {
    public static final int LEFT_BLUE = 0;
    public static final int RIGHT_BLUE = 1;
    public static final int LEFT_RED = 2;
    public static final int RIGHT_RED = 3;

    public int mode = LEFT_BLUE;
    public boolean isExecutingPurplePath = false;
    public boolean isExecutingYellowPath = false;
    public boolean isExecutingBackstagePath = false;

    public ChassisOld chassis;
    public VisionPipeline vision;

    public void runSetup(int mode, ChassisOld chassis) {
        this.mode = mode;
        this.chassis = chassis;
        try {
            //vision = new VisionPipeline(chassis);
            this.chassis.camera.setPipeline(vision);
            this.chassis.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    MainAutoOld.this.chassis.camera.startStreaming(VisionPipeline.IMAGE_WIDTH, VisionPipeline.IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
            while (vision.route == -1) {
                // do absolutely nothing
            }
            this.chassis.camera.closeCameraDeviceAsync(() -> {
                // well also do nothing here
            });
            isExecutingPurplePath = true;
        } catch (Exception e) {
            // Do nothing
        }
    }

    public void runLoop() {
        if (isExecutingPurplePath) {
            executePurplePath();
        } else if (isExecutingYellowPath) {
            executeYellowPath();
        } else if (isExecutingBackstagePath) {
            executeBackstagePath();
        }
    }

    public void executePurplePath() {
        switch (vision.route) {
            case VisionPipeline.LEFT_PATH:
                chassis.moveChassisOld(1, Game.TILE_SIZE);
                chassis.turnChassisOld(-1, 45);
                break;
            case VisionPipeline.CENTER_PATH:
                chassis.moveChassisOld(1, Game.TILE_SIZE);
                break;
            case VisionPipeline.RIGHT_PATH:
                chassis.moveChassisOld(1, Game.TILE_SIZE);
                chassis.turnChassisOld(1, 45);
                break;
        }
        // back drive roller
        chassis.turnRollerAuto(-0.25, 1);

        isExecutingPurplePath = false;
        isExecutingYellowPath = true;
    }

    // April tag contour detection necessary
    public void executeYellowPath() {
        switch (mode) {
            case LEFT_BLUE:
                switch (vision.route) {
                    case VisionPipeline.LEFT_PATH:
                        chassis.turnChassisOld(1, 135);
                        break;
                    case VisionPipeline.CENTER_PATH:
                        chassis.turnChassisOld(1, 90);
                        break;
                    case VisionPipeline.RIGHT_PATH:
                        chassis.turnChassisOld(1, 45);
                        break;
                }
                chassis.moveChassisOld(1, Game.TILE_SIZE);
                break;
            case RIGHT_BLUE:
                switch (vision.route) {
                    case VisionPipeline.LEFT_PATH:
                        chassis.turnChassisOld(1, 135);
                        break;
                    case VisionPipeline.CENTER_PATH:
                        chassis.turnChassisOld(1, 90);
                        break;
                    case VisionPipeline.RIGHT_PATH:
                        chassis.turnChassisOld(1, 45);
                        break;
                }
                chassis.moveChassisOld(3, Game.TILE_SIZE);
                break;
            case LEFT_RED:
                switch (vision.route) {
                    case VisionPipeline.LEFT_PATH:
                        chassis.turnChassisOld(-1, 45);
                        break;
                    case VisionPipeline.CENTER_PATH:
                        chassis.turnChassisOld(-1, 90);
                        break;
                    case VisionPipeline.RIGHT_PATH:
                        chassis.turnChassisOld(-1, 135);
                        break;
                }
                chassis.moveChassisOld(3, Game.TILE_SIZE);
                break;
            case RIGHT_RED:
                switch (vision.route) {
                    case VisionPipeline.LEFT_PATH:
                        chassis.turnChassisOld(-1, 45);
                        break;
                    case VisionPipeline.CENTER_PATH:
                        chassis.turnChassisOld(-1, 90);
                        break;
                    case VisionPipeline.RIGHT_PATH:
                        chassis.turnChassisOld(-1, 135);
                        break;
                }
                chassis.moveChassisOld(1, Game.TILE_SIZE);
                break;
        }
        chassis.turnArmAuto(1, 120);
        chassis.turnFlap(0.5, 3);
        chassis.turnArmAuto(-1, 120);
        isExecutingYellowPath = false;
        isExecutingBackstagePath = true;
    }

    public void executeBackstagePath() {
        switch (mode) {
            case LEFT_BLUE:
            case RIGHT_BLUE:
                chassis.turnChassisOld(-1, 90);
                break;
            case LEFT_RED:
            case RIGHT_RED:
                chassis.turnChassisOld(1, 90);
                break;
        }
        chassis.moveChassisOld(-1, Game.TILE_SIZE);
        isExecutingBackstagePath = false;
    }
}
