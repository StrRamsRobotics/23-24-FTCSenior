package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Init {
    public enum Team {
        RED, BLUE
    }
    public enum Side {
        BOARD, EDGE
    }
    public static OpenCvWebcam camera;
    public static SampleMecanumDrive drive;
    public static final double METRES_TO_INCH = 39.3701;
    public static final double CM_TO_INCH = 0.393701;

    public static DcMotor frontLeftDrive;
    public static DcMotor frontRightDrive;
    public static DcMotor backLeftDrive;
    public static DcMotor backRightDrive;
    public static DcMotor leftSlide;
    public static DcMotor rightSlide;
    public static DcMotorEx leftClimb;
    public static DcMotorEx rightClimb;

    public static CRServo intake;
    public static Servo leftOuttake, rightOuttake, intakeTilt, plane;
    public static Outtake out;
    public static Intake in;
    public static Climb climb;
    public static AprilTagDriver april;
    public static PropPipeline prop;
    public static void init(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        FtcDashboard.start(null);

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightClimb = hardwareMap.get(DcMotorEx.class, "rightClimb");
        leftClimb = hardwareMap.get(DcMotorEx.class, "leftClimb");

        intake = hardwareMap.get(CRServo.class, "intake");
        leftOuttake = hardwareMap.get(Servo.class, "leftOuttake");
        rightOuttake = hardwareMap.get(Servo.class, "rightOuttake");
        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        //plane = hardwareMap.get(Servo.class, "plane"); todo readd once plane plugged in

//        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftClimb.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOuttake.setDirection(Servo.Direction.REVERSE);

        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightClimb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftClimb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        out = new Outtake(hardwareMap);
        in = new Intake(hardwareMap);
        climb = new Climb(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                april = new AprilTagDriver(camera);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }
            @Override
            public void onError(int errorCode) {}
        });
    }
}
