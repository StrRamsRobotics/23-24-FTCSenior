package org.firstinspires.ftc.teamcode.opmodes.base;

import org.firstinspires.ftc.teamcode.opmodes.auto.actions.VisionAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoPath;
import org.firstinspires.ftc.teamcode.opmodes.auto.classes.AutoPoint;

import java.util.ArrayList;

public abstract class BaseAuto extends BaseOpmode {
    public int route = -1;
    public VisionAction visionAction = null;
    public ArrayList<AutoPoint> points = new ArrayList<>();
    public AutoPath path;
    public AutoPath autoPath;

    public abstract void createPoints();
}
