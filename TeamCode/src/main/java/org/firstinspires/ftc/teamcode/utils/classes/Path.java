package org.firstinspires.ftc.teamcode.utils.classes;

import java.util.ArrayList;

public class Path {
    public ArrayList<Point> points;

    public Path(ArrayList<Point> points) {
        this.points = points;
    }

    public Path(Path c) {
        this.points = c.points;
    }

    public Path() {
        this.points.add(new Point());
    }

    public Path addPoint(Point p) {
        points.add(p);
        return this;
    }

    public Path removePoint(Point p) {
        points.remove(p);
        return this;
    }

    public Path setPoint(int index, Point p) {
        points.set(index, p);
        return this;
    }
}
