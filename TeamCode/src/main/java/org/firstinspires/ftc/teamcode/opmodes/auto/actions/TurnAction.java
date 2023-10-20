package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoLine;
import org.firstinspires.ftc.teamcode.wrappers.Chassis;

public class TurnAction extends AutoAction {
    public double power;
    public AutoLine line;
    public double angle;
    public double arcLength;
    public long startTime;

    public TurnAction(Chassis chassis, double power, AutoLine line) {
        super(chassis);
//        this.power = power * Math.signum(line.getAngle());
        this.power = power;
        this.angle = Math.abs(line.getAngle());
        this.arcLength = Math.toRadians(angle) * Chassis.ROBOT_WIDTH / 2;
        this.startTime = System.currentTimeMillis();
    }

    public TurnAction(Chassis chassis, double power, double angle) {
        super(chassis);
        this.power = power;
        this.angle = angle;
        this.arcLength = Math.toRadians(angle) * Chassis.ROBOT_WIDTH / 2;
    }

    public void tick() {
        if (!this.isInitialized) {
            this.startTime = System.currentTimeMillis();
            this.isInitialized = true;
        }
        chassis.logHelper.addData("Running", "TurnAction");
        long currentTime = System.currentTimeMillis();
        double distance = (currentTime - startTime) * Chassis.MOVE_DISTANCE_PER_SECOND / 1000.0;
        chassis.logHelper.addData("Angle", angle);
        chassis.logHelper.addData("Distance", distance);
        chassis.logHelper.addData("ArcLength", arcLength);
        if (distance < arcLength) {
            // mecanum
            chassis.fr.setPower(power);
            chassis.fl.setPower(-power);
            chassis.br.setPower(power);
            chassis.bl.setPower(-power);
        } else {
            chassis.fr.setPower(0);
            chassis.fl.setPower(0);
            chassis.br.setPower(0);
            chassis.bl.setPower(0);
            active = false;
        }
    }
}
