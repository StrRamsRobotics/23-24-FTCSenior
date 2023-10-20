package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoAction;
import org.firstinspires.ftc.teamcode.wrappers.Chassis;

public class MoveAction extends AutoAction {
    public double power;

    public MoveAction (Chassis chassis, double power, boolean reverse) {
        super(chassis);
        if (reverse) {
            this.power = -power;
        }
        else {
            this.power = power;
        }
    }

    public MoveAction (Chassis chassis, double power) {
        super(chassis);
        this.power = power;
    }

    public void tick() {
        chassis.logHelper.addData("Running", "MoveAction");
        if (!isInitialized) {
            chassis.fr.setPower(power);
            chassis.fl.setPower(power);
            if (!Chassis.TWO_WHEELED) {
                chassis.br.setPower(power);
                chassis.bl.setPower(power);
            }
            isInitialized = true;
        }
        active = false;
    }
}
