package org.firstinspires.ftc.teamcode.utils.classes;

public class Point {
    public double x;
    public double y;

    public static final double APPROXIMATE_THRESHOLD = 0.1;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(Point p) {
        this.x = p.x;
        this.y = p.y;
    }

    public Point() {
        this.x = 0;
        this.y = 0;
    }

    public double distanceTo(Point p) {
        return Math.sqrt(Math.pow(p.x - this.x, 2) + Math.pow(p.y - this.y, 2));
    }

    public double angleTo(Point p) {
        return Math.atan2(p.y - this.y, p.x - this.x);
    }

    public double angleTo(Point p, double heading) {
        return Math.atan2(p.y - this.y, p.x - this.x) - heading;
    }

    public double angleTo(Point p, double heading, double offset) {
        return Math.atan2(p.y - this.y, p.x - this.x) - heading + offset;
    }

    public boolean equals(Point p) {
        return this.x == p.x && this.y == p.y;
    }

    public boolean approximatelyEquals(Point p) {
        return distanceTo(p) < APPROXIMATE_THRESHOLD;
    }
}
