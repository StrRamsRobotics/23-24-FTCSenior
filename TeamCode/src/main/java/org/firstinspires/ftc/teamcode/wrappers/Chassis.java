package org.firstinspires.ftc.teamcode.wrappers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.helpers.LogHelper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class Chassis {
    public static String IMU_NAME = "imu";
    public static String FR_NAME = "fr";
    public static String FL_NAME = "fl";
    public static String BR_NAME = "br";
    public static String BL_NAME = "bl";
    public static String ARM_NAME = "arm";
    public static String PIVOT_NAME = "pivot";
    public static String ROLLER_NAME = "roller";
    public static String FLAP_NAME = "flap";
    public static String CAMERA_NAME = "camera";

    public static final boolean TANK_DRIVE = false;
    public static final boolean TWO_WHEELED = false;
    public static final boolean HAS_CHASSIS_ENCODERS = false;

    public static final boolean HAS_ARM = true;
    public static final boolean HAS_PIVOT = false;
    public static final boolean HAS_ROLLER = true;
    public static final boolean HAS_FLAP = true;

    public static final double MOVE_POWER = 1;
    public static final double ARM_POWER = 1;
    public static final double PIVOT_POWER = 1;
    public static final double ROLLER_POWER = 1;

    public static final int ROBOT_WIDTH = 18; // inches
    public static final int ROBOT_LENGTH = 18; // inchjes <- written by trent lol
    public static final double MOVE_DISTANCE_PER_SECOND = 24 * MOVE_POWER; // inches
    public static final int CORE_HEX_TICKS_PER_REV = 288;
    public static final int ROLLER_RADIUS = 2; // inches

    public DcMotorEx fr, fl, br, bl;
    public DcMotorEx arm, pivot, roller;
    public Servo flap;
    public BNO055IMU imu;
    public OpenCvCamera camera;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    public LogHelper logHelper = new LogHelper(this);

    public Chassis(HardwareMap hardwareMap, Telemetry telemetry) {
        initializeUtils(hardwareMap, telemetry);
        initializeIMU();
        initializeMotors();
        initializeCamera();
    }

    public void initializeUtils(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, IMU_NAME);
        imu.initialize(parameters);
    }

    public void initializeMotors() {
        fr = hardwareMap.get(DcMotorEx.class, FR_NAME);
//        fr.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.FORWARD); // this is correct
        fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        fl.setDirection(DcMotorEx.Direction.FORWARD); // this is correct
        if (!TWO_WHEELED) {
            br = hardwareMap.get(DcMotorEx.class, BR_NAME);
//            br.setDirection(DcMotorEx.Direction.REVERSE);
            br.setDirection(DcMotorEx.Direction.FORWARD); // this is correct
            bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
//            bl.setDirection(DcMotorEx.Direction.FORWARD);
            bl.setDirection(DcMotorEx.Direction.REVERSE); // this is correct
        }

        if (HAS_ARM) {
            arm = hardwareMap.get(DcMotorEx.class, ARM_NAME);
            arm.setDirection(DcMotorEx.Direction.FORWARD);
        }
        if (HAS_PIVOT) {
            pivot = hardwareMap.get(DcMotorEx.class, PIVOT_NAME);
            pivot.setDirection(DcMotorEx.Direction.FORWARD);
        }
        if (HAS_ROLLER) {
            roller = hardwareMap.get(DcMotorEx.class, ROLLER_NAME);
            roller.setDirection(DcMotorEx.Direction.FORWARD);
        }
        if (HAS_FLAP) {
            flap = hardwareMap.get(Servo.class, FLAP_NAME);
            flap.setDirection(Servo.Direction.FORWARD);
        }

        if (HAS_CHASSIS_ENCODERS) {
            fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            if (!TWO_WHEELED) {
                br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }

            fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            if (!TWO_WHEELED) {
                br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, CAMERA_NAME), cameraMonitorViewId);
    }

    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle_value = -AngleUnit.DEGREES.fromUnit(orientation.angleUnit, orientation.firstAngle);
        if (Math.abs(angle_value)%360>180) {
            return Math.signum(angle_value)*(Math.abs(angle_value)%360-360);
        } else {
            return Math.signum(angle_value)*(Math.abs(angle_value)%360);
        }
    }
}
