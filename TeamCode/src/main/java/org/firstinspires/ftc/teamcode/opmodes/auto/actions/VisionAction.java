package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.pipelines.VisionPipeline;
import org.firstinspires.ftc.teamcode.wrappers.Chassis;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class VisionAction extends AutoAction {
    public VisionPipeline visionPipeline;
    public int route;
    public boolean isBlue;

    public VisionAction(Chassis chassis, boolean isBlue) {
        super(chassis);
        route = -1;
        this.isBlue = isBlue;
    }

    public void tick() {
        chassis.logHelper.addData("Running", "VisionAction");
        if (!isInitialized) {
            try {
                visionPipeline = new VisionPipeline(isBlue);
                chassis.camera.setPipeline(visionPipeline);
                chassis.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        chassis.camera.startStreaming(VisionPipeline.IMAGE_WIDTH, VisionPipeline.IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                        chassis.ftcDashboard.startCameraStream(chassis.camera, 30);
                    }
                    @Override
                    public void onError(int errorCode) {
                    }
                });
            } catch (Exception e) {}
            isInitialized = true;
        }
        route = visionPipeline.route;
        if (visionPipeline.maxAreaIV != null) {
            chassis.logHelper.addData("Max Area", visionPipeline.maxAreaIV.value);
        }
        if (visionPipeline.center != null) {
            chassis.logHelper.addData("Center X", visionPipeline.center.x);
        }
        chassis.logHelper.addData("VisionAction Route", route);
        if (route != -1) {
            chassis.camera.closeCameraDeviceAsync(() -> {
            });
            active = false;
        }
    }
}
