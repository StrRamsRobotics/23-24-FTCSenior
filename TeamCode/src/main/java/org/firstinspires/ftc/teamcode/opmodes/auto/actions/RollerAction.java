package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoAction;
import org.firstinspires.ftc.teamcode.wrappers.Chassis;

public class RollerAction extends AutoAction {
    public double power;

    public RollerAction(Chassis chassis, double power) {
        super(chassis);
        this.power = power;
    }

    public void tick() {
        chassis.logHelper.addData("Running", "RollerAction");
        chassis.logHelper.addData("Power", power);
        if (!isInitialized) {
            chassis.roller.setPower(power);
            isInitialized = true;
        }
        active = false;
    }
}
