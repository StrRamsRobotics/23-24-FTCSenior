package org.firstinspires.ftc.teamcode.opmodes.auto.classes;

import org.firstinspires.ftc.teamcode.opmodes.auto.actions.MoveAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.actions.TurnAction;
import org.firstinspires.ftc.teamcode.wrappers.Chassis;

import java.util.ArrayList;

public class AutoPath {
    public Chassis chassis;

    public AutoPoint currentPoint;
    public AutoLine currentLine;
    public AutoPoint activePoint;
    public AutoPoint nextPoint;
    public double activeDistanceToNext;

    public long prevTime;

    public ArrayList<AutoPoint> autoPoints;
    public ArrayList<AutoLine> lines;
    public boolean active = true;

    public AutoPath(Chassis chassis, ArrayList<AutoPoint> autoPoints) {
        this.chassis = chassis;
        this.autoPoints = autoPoints;
        this.lines = new ArrayList<>();
        for (AutoPoint autoPoint : autoPoints) {
            if (autoPoints.indexOf(autoPoint) != autoPoints.size() - 1) {
                AutoLine line = getConnectingLine(autoPoint);
                if (autoPoint.heading == Double.MAX_VALUE) {
                    autoPoint.heading = line.getHeading();
                }
                lines.add(line);
                autoPoint
                        .addAutoAction(0, new MoveAction(chassis, 0, autoPoint.isReverse))
                        .addAutoAction(new TurnAction(chassis, Chassis.MOVE_POWER, line))
                        .addAutoAction(new MoveAction(chassis, Chassis.MOVE_POWER, autoPoint.isReverse));
            }
        }
        if (autoPoints.size() > 0) {
            currentPoint = autoPoints.get(0);
            activePoint = currentPoint;
        }
        if (autoPoints.size() > 1) {
            nextPoint = autoPoints.get(1);
            activeDistanceToNext = nextPoint.distanceTo(currentPoint);
        }
        if (lines.size() > 0) currentLine = lines.get(0);
        prevTime = System.currentTimeMillis();
        active = true;
    }

    public AutoLine getConnectingLine(AutoPoint autoPoint) {
        if (autoPoints.get(autoPoints.indexOf(autoPoint) + 1) != null) {
            return new AutoLine(autoPoint, autoPoints.get(autoPoints.indexOf(autoPoint) + 1));
        }
        else {
            return null;
        }
    }

    public void tick() {
        long currentTime = System.currentTimeMillis();
        double stepDistance = (currentTime - prevTime) * Chassis.MOVE_DISTANCE_PER_SECOND / 1000.0;

        if (activePoint != null && currentPoint != null) {
            chassis.logHelper.addData("Number of points", autoPoints.size());;
            chassis.logHelper.addData("Active point index", autoPoints.indexOf(activePoint));
            chassis.logHelper.addData("Active point x", activePoint.x);
            chassis.logHelper.addData("Active point y", activePoint.y);
            chassis.logHelper.addData("Active point heading", activePoint.heading);
            chassis.logHelper.addData("Line heading", currentLine.getHeading());
            chassis.logHelper.addData("Active point distance to current", activePoint.distanceTo(currentPoint));
            chassis.logHelper.addData("Active point distance to next", activeDistanceToNext);
            activePoint.tick();
            chassis.logHelper.addData("Active point action index", activePoint.autoActions.indexOf(activePoint.currentAutoAction));
            if (activePoint.currentAutoAction != null) {
                chassis.logHelper.addData("Active point action active", activePoint.currentAutoAction.active);
            }
            chassis.logHelper.addData("Active point active", activePoint.active);
            if (!activePoint.active) {
                currentPoint = currentLine.getNextPoint(currentPoint, stepDistance);
                double currentDistanceToNext = currentPoint.distanceTo(activePoint);
                chassis.logHelper.addData("Slope", currentLine.slope);
                chassis.logHelper.addData("Step distance", stepDistance);
                chassis.logHelper.addData("Current point x", currentPoint.x);
                chassis.logHelper.addData("Current point y", currentPoint.y);
                chassis.logHelper.addData("Current point heading", currentPoint.heading);
                if (currentDistanceToNext > activeDistanceToNext) {
                    chassis.logHelper.addData("Next index", autoPoints.indexOf(nextPoint) + 1);
                    if (autoPoints.indexOf(nextPoint) + 1 < autoPoints.size()) {
                        activePoint = nextPoint;
                        nextPoint = autoPoints.get(autoPoints.indexOf(activePoint) + 1);
                        activeDistanceToNext = activePoint.distanceTo(nextPoint);
                        currentPoint = activePoint;
                        currentLine = lines.get(autoPoints.indexOf(activePoint));
                    }
                    else {
                        active = false;
                        chassis.fr.setPower(0);
                        chassis.fl.setPower(0);
                        if (!Chassis.TWO_WHEELED) {
                            chassis.br.setPower(0);
                            chassis.bl.setPower(0);
                        }
                    }
                }
            }
        }
        else {
            active = false;
            chassis.fr.setPower(0);
            chassis.fl.setPower(0);
            if (!Chassis.TWO_WHEELED) {
                chassis.br.setPower(0);
                chassis.bl.setPower(0);
            }
        }

        prevTime = currentTime;
        chassis.logHelper.update();
    }

//    public AutoPath tick() {
//        long currentTime = System.currentTimeMillis();
//        double stepDistance = (currentTime - prevTime) * Chassis.MOVE_DISTANCE_PER_SECOND / 1000.0;
//        boolean activePointRunning = true;
//
//        if (activePointRunning && activePoint != null) {
//            chassis.logHelper.addData("Number of points", autoPoints.size());
//            chassis.logHelper.addData("Active point index", autoPoints.indexOf(activePoint));
//            chassis.logHelper.addData("Active point x", activePoint.x);
//            chassis.logHelper.addData("Active point y", activePoint.y);
//            chassis.logHelper.addData("Active point heading", activePoint.heading);
//            AutoPoint point = activePoint.tick();
//            if (point == null) {
//                activePointRunning = false;
//            }
//            else {
//                activePoint = point;
//            }
//        }
//        if (!activePointRunning) {
//            currentPoint = currentLine.getNextPoint(currentPoint, stepDistance);
//            chassis.logHelper.addData("Slope", currentLine.slope);
//            chassis.logHelper.addData("Step distance", stepDistance);
//            chassis.logHelper.addData("Current point x", currentPoint.x);
//            chassis.logHelper.addData("Current point y", currentPoint.y);
//            chassis.logHelper.addData("Current point heading", currentPoint.heading);
//            chassis.logHelper.addData("Active point distance to current", activePoint.distanceTo(currentPoint));
//            chassis.logHelper.addData("Active point distance to next", activeDistanceToNext);
//            if (
//                    autoPoints.indexOf(currentPoint) + 1 < autoPoints.size() &&
//                    activePoint.distanceTo(currentPoint) > activeDistanceToNext
//            ) {
//                activePoint = nextPoint;
//                chassis.logHelper.addData("New active point", autoPoints.indexOf(activePoint));
//                chassis.logHelper.addData("Active point x", activePoint.x);
//                chassis.logHelper.addData("Active point y", activePoint.y);
//                nextPoint = autoPoints.get(autoPoints.indexOf(activePoint) + 1);
//                activeDistanceToNext = activePoint.distanceTo(nextPoint);
//            }
//        }
//        prevTime = currentTime;
//        chassis.logHelper.update();
//        return this;
//    }
}
