package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.base.BaseTeleop;
import org.firstinspires.ftc.teamcode.utils.helpers.MathHelper;
import org.firstinspires.ftc.teamcode.wrappers.Chassis;

@TeleOp(name = "TeleOp")
public class Teleop extends BaseTeleop {
    public static final double JOYSTICK_DEADZONE = 0.025;

    @Override
    public void runSetup() {
        chassis.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void runLoop() {
        controlGP1();
        controlGP2();
    }

    public void controlGP1() {
        double lx = gamepad1.left_stick_x, ly = gamepad1.left_stick_y, rx = gamepad1.right_stick_x, ry = gamepad1.right_stick_y;
        double lxp = Math.pow(lx, 2), lyp = Math.pow(ly, 2), rxp = Math.pow(rx, 2), ryp = Math.pow(ry, 2);
        boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
        chassis.fr.setPower(MathHelper.deadzone(-lxp + -lyp - rxp, JOYSTICK_DEADZONE));
        chassis.fl.setPower(MathHelper.deadzone(lxp + -lyp - rxp, JOYSTICK_DEADZONE));
        chassis.br.setPower(MathHelper.deadzone(-lxp + lyp - rxp, JOYSTICK_DEADZONE));
        chassis.bl.setPower(MathHelper.deadzone(lxp + lyp - rxp, JOYSTICK_DEADZONE));
    }

    public void controlGP2() {
        double lx = gamepad2.left_stick_x, ly = gamepad2.left_stick_y, rx = gamepad2.right_stick_x, ry = gamepad2.right_stick_y;
        double lxp = Math.pow(lx, 2), lyp = Math.pow(ly, 2), rxp = Math.pow(rx, 2), ryp = Math.pow(ry, 2);
        boolean a = gamepad2.a, b = gamepad2.b, x = gamepad2.x, y = gamepad2.y;
        chassis.arm.setPower(MathHelper.deadzone(lyp, JOYSTICK_DEADZONE));
        chassis.roller.setPower(MathHelper.deadzone(ryp, JOYSTICK_DEADZONE));
    }
}
