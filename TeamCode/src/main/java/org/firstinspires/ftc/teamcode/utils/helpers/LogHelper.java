package org.firstinspires.ftc.teamcode.utils.helpers;

import org.firstinspires.ftc.teamcode.wrappers.Chassis;

public class LogHelper {
    Chassis chassis;

    public LogHelper(Chassis chassis) {
        this.chassis = chassis;
    }

    public void addData(String key, Object value) {
        System.out.println(value);
        this.chassis.telemetry.addData(key, value);
        this.chassis.ftcDashboard.getTelemetry().addData(key, value);
    }

    public void update() {
        this.chassis.telemetry.update();
        this.chassis.ftcDashboard.getTelemetry().update();
    }
}
