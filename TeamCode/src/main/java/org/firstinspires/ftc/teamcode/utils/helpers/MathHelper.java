package org.firstinspires.ftc.teamcode.utils.helpers;

public class MathHelper {
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double deadzone(double value, double deadzone) {
        return Math.abs(value) < deadzone ? 0 : value;
    }

    public static double radius(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    public static double angle(double x, double y) {
        return Math.atan2(y, x);
    }

    public static double toHeading(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            return angle - 180;
        }
        else {
            return angle;
        }
    }
}
