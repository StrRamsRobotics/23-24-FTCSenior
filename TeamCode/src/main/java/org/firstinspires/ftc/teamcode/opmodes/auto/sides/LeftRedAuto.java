package org.firstinspires.ftc.teamcode.opmodes.auto.sides;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.actions.AprilTagAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.actions.ArmAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.actions.FlapAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.actions.RollerAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.actions.TurnAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.actions.VisionAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.actions.WaitAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoPath;
import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoPoint;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseAuto;
import org.firstinspires.ftc.teamcode.utils.classes.Point;
import org.firstinspires.ftc.teamcode.wrappers.Chassis;
import org.firstinspires.ftc.teamcode.wrappers.Game;

import java.util.ArrayList;

@Autonomous(name="leftRedAuto")
public class LeftRedAuto extends BaseAuto {
    @Override
    public void runSetup() {
        visionAction = new VisionAction(chassis, false);
        visionAction.tick(); // to init camera stream before 30 seconds
    }
    @Override
    public void createPoints() {
        // based off of front of robot
        points.add(new AutoPoint(new Point(0, 2.5 * Game.TILE_SIZE), new ArrayList<>(), true));
        ArrayList<AutoAction> purpleActions = new ArrayList<>();
        ArrayList<AutoAction> yellowActions = new ArrayList<>();
        yellowActions.add(new AprilTagAction(chassis, Game.RED_TEAM, route));
        if (Chassis.HAS_ARM) yellowActions.add(new ArmAction(chassis, Chassis.ARM_POWER, 120));
        if (Chassis.HAS_FLAP) yellowActions.add(new FlapAction(chassis, 1));
        yellowActions.add(new WaitAction(chassis, 1000));
        if (Chassis.HAS_FLAP) yellowActions.add(new FlapAction(chassis, 0));
        if (Chassis.HAS_ARM) yellowActions.add(new ArmAction(chassis, Chassis.ARM_POWER, -120));
        switch(route) {
            case 0:
                purpleActions.add(new TurnAction(chassis, 1, 45));
                if (Chassis.HAS_ROLLER) purpleActions.add(new RollerAction(chassis, -Chassis.ROLLER_POWER));
                purpleActions.add(new WaitAction(chassis, 1000));
                if (Chassis.HAS_ROLLER) purpleActions.add(new RollerAction(chassis, 0));
                points.add(new AutoPoint(new Point(4 * Game.TILE_SIZE, 4.5 * Game.TILE_SIZE), purpleActions, true));
                points.add(new AutoPoint(new Point(3.5 * Game.TILE_SIZE, 4.5 * Game.TILE_SIZE), purpleActions, true));
                points.add(new AutoPoint(new Point(3.5 * Game.TILE_SIZE, 2 * Game.TILE_SIZE), purpleActions, true));
                points.add(new AutoPoint(new Point(4.75 * Game.TILE_SIZE, 1.5 * Game.TILE_SIZE), yellowActions, true));
                break;
            case 2:
                purpleActions.add(new TurnAction(chassis, 1, -45));
                if (Chassis.HAS_ROLLER) purpleActions.add(new RollerAction(chassis, -Chassis.ROLLER_POWER));
                purpleActions.add(new WaitAction(chassis,1000));
                if (Chassis.HAS_ROLLER) purpleActions.add(new RollerAction(chassis, 0));
                points.add(new AutoPoint(new Point(4 * Game.TILE_SIZE, 4.5 * Game.TILE_SIZE), purpleActions, true));
                points.add(new AutoPoint(new Point(3.5 * Game.TILE_SIZE, 4.5 * Game.TILE_SIZE), purpleActions, true));
                points.add(new AutoPoint(new Point(3.5 * Game.TILE_SIZE, 2 * Game.TILE_SIZE), purpleActions, true));
                points.add(new AutoPoint(new Point(4.25 * Game.TILE_SIZE, 1.5 * Game.TILE_SIZE), yellowActions, true));
                break;
            case 1:
            default:
                if (Chassis.HAS_ROLLER) purpleActions.add(new RollerAction(chassis, -Chassis.ROLLER_POWER));
                purpleActions.add(new WaitAction(chassis,1000));
                if (Chassis.HAS_ROLLER) purpleActions.add(new RollerAction(chassis, 0));
                points.add(new AutoPoint(new Point(4 * Game.TILE_SIZE, 4.5 * Game.TILE_SIZE), purpleActions, true));
                points.add(new AutoPoint(new Point(3.5 * Game.TILE_SIZE, 4.5 * Game.TILE_SIZE), purpleActions, true));
                points.add(new AutoPoint(new Point(3.5 * Game.TILE_SIZE, 2 * Game.TILE_SIZE), purpleActions, true));
                points.add(new AutoPoint(new Point(4.5 * Game.TILE_SIZE, 1.5 * Game.TILE_SIZE), yellowActions, true));
                break;
        }
        points.add(new AutoPoint(new Point(5.5 * Game.TILE_SIZE, Game.TILE_SIZE), new ArrayList<>(), true));
        path = new AutoPath(chassis, points);
    }

    @Override
    public void runLoop() {
        if (route == -1) {
            visionAction.tick();
            if (!visionAction.active) {
                route = visionAction.route;
            }
            chassis.logHelper.update();
        }
        else {
            if (points.size() == 0) {
                createPoints();
            }
            chassis.logHelper.addData("Route", route);
            if (path.active) {
                path.tick();
            }
        }
    }
}