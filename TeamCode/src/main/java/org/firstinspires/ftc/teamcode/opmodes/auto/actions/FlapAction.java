package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoAction;
import org.firstinspires.ftc.teamcode.wrappers.Chassis;

public class FlapAction extends AutoAction {
    public double position;

    public FlapAction(Chassis chassis, double position) {
        super(chassis);
        this.position = position;
    }

    public void tick() {
        chassis.logHelper.addData("Running", "FlapAction");
        chassis.logHelper.addData("Direction", chassis.flap.getDirection());
        chassis.logHelper.addData("Position", position);
        if (!this.isInitialized) {
            chassis.flap.setPosition(position);
            isInitialized = true;
        }
        active = false;
    }
}
