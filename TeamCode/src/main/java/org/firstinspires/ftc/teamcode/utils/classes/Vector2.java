package org.firstinspires.ftc.teamcode.utils.classes;

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2(Vector2 v) {
        this.x = v.x;
        this.y = v.y;
    }

    public Vector2() {
        this.x = 0;
        this.y = 0;
    }

    public Vector2 add(Vector2 v) {
        return new Vector2(this.x + v.x, this.y + v.y);
    }

    public Vector2 sub(Vector2 v) {
        return new Vector2(this.x - v.x, this.y - v.y);
    }

    public Vector2 mul(double s) {
        return new Vector2(this.x * s, this.y * s);
    }

    public Vector2 div(double s) {
        return new Vector2(this.x / s, this.y / s);
    }

    public double dot(Vector2 v) {
        return this.x * v.x + this.y * v.y;
    }

    public double cross(Vector2 v) {
        return this.x * v.y - this.y * v.x;
    }

    public double magnitude() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public Vector2 normalize() {
        return this.div(this.magnitude());
    }

    public Vector2 rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector2(this.x * cos - this.y * sin, this.x * sin + this.y * cos);
    }

    public Vector2 rotateAround(Vector2 point, double angle) {
        return this.sub(point).rotate(angle).add(point);
    }

    public Vector2 project(Vector2 v) {
        return v.mul(this.dot(v) / v.dot(v));
    }

    public Vector2 projectOnto(Vector2 v) {
        return this.mul(v.dot(this) / this.dot(this));
    }

    public Vector2 reflect(Vector2 normal) {
        return this.sub(this.project(normal).mul(2));
    }

    public Vector2 reflectAcross(Vector2 v) {
        return this.sub(this.projectOnto(v).mul(2));
    }

    public Vector2 lerp(Vector2 v, double t) {
        return this.add(v.sub(this).mul(t));
    }

    public Vector2 slerp(Vector2 v, double t) {
        double angle = Math.acos(this.dot(v) / (this.magnitude() * v.magnitude()));
        return this.rotate(angle * t);
    }

    public Vector2 nlerp(Vector2 v, double t) {
        return this.lerp(v, t).normalize();
    }

    public Vector2 nslerp(Vector2 v, double t) {
        return this.slerp(v, t).normalize();
    }

    public Vector2 abs() {
        return new Vector2(Math.abs(this.x), Math.abs(this.y));
    }

    public Vector2 min(Vector2 v) {
        return new Vector2(Math.min(this.x, v.x), Math.min(this.y, v.y));
    }

    public Vector2 max(Vector2 v) {
        return new Vector2(Math.max(this.x, v.x), Math.max(this.y, v.y));
    }

    public Vector2 clamp(Vector2 min, Vector2 max) {
        return this.max(min).min(max);
    }

    public Vector2 deadzone(double deadzone) {
        return new Vector2(Math.abs(this.x) < deadzone ? 0 : this.x, Math.abs(this.y) < deadzone ? 0 : this.y);
    }
}
