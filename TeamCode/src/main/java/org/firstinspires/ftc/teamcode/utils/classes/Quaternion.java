package org.firstinspires.ftc.teamcode.utils.classes;

public class Quaternion {
    public double w;
    public double x;
    public double y;
    public double z;

    public Quaternion(double w, double x, double y, double z) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Quaternion(Quaternion q) {
        this.w = q.w;
        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
    }

    public Quaternion() {
        this.w = 1;
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    public Quaternion add(Quaternion q) {
        return new Quaternion(this.w + q.w, this.x + q.x, this.y + q.y, this.z + q.z);
    }

    public Quaternion sub(Quaternion q) {
        return new Quaternion(this.w - q.w, this.x - q.x, this.y - q.y, this.z - q.z);
    }

    public Quaternion mul(double s) {
        return new Quaternion(this.w * s, this.x * s, this.y * s, this.z * s);
    }

    public Quaternion div(double s) {
        return new Quaternion(this.w / s, this.x / s, this.y / s, this.z / s);
    }

    public Quaternion mul(Quaternion q) {
        return new Quaternion(
            this.w * q.w - this.x * q.x - this.y * q.y - this.z * q.z,
            this.w * q.x + this.x * q.w + this.y * q.z - this.z * q.y,
            this.w * q.y - this.x * q.z + this.y * q.w + this.z * q.x,
            this.w * q.z + this.x * q.y - this.y * q.x + this.z * q.w
        );
    }

    public Quaternion conjugate() {
        return new Quaternion(this.w, -this.x, -this.y, -this.z);
    }

    public double magnitude() {
        return Math.sqrt(this.w * this.w + this.x * this.x + this.y * this.y + this.z * this.z);
    }

    public Quaternion normalize() {
        return this.div(this.magnitude());
    }

    public Quaternion inverse() {
        return this.conjugate().div(this.magnitude());
    }

    public Vector3 rotate(Vector3 v) {
        Quaternion q = new Quaternion(0, v.x, v.y, v.z);
        Quaternion qInv = this.inverse();
        Quaternion qRot = this.mul(q).mul(qInv);
        return new Vector3(qRot.x, qRot.y, qRot.z);
    }

    public Vector3 toEuler() {
        double roll = Math.atan2(2 * (this.w * this.x + this.y * this.z), 1 - 2 * (this.x * this.x + this.y * this.y));
        double pitch = Math.asin(2 * (this.w * this.y - this.z * this.x));
        double yaw = Math.atan2(2 * (this.w * this.z + this.x * this.y), 1 - 2 * (this.y * this.y + this.z * this.z));
        return new Vector3(roll, pitch, yaw);
    }

    public Quaternion fromEuler(Vector3 v) {
        double cy = Math.cos(v.z * 0.5);
        double sy = Math.sin(v.z * 0.5);
        double cp = Math.cos(v.y * 0.5);
        double sp = Math.sin(v.y * 0.5);
        double cr = Math.cos(v.x * 0.5);
        double sr = Math.sin(v.x * 0.5);

        return new Quaternion(
                cy * cp * cr + sy * sp * sr,
                cy * cp * sr - sy * sp * cr,
                sy * cp * sr + cy * sp * cr,
                sy * cp * cr - cy * sp * sr
        );
    }

    public static Quaternion fromAxisAngle(Vector3 axis, double angle) {
        double halfAngle = angle / 2;
        double sin = Math.sin(halfAngle);
        double cos = Math.cos(halfAngle);
        return new Quaternion(cos, axis.x * sin, axis.y * sin, axis.z * sin);
    }

    @Override
    public String toString() {
        return String.format("Quaternion(%f, %f, %f, %f)", this.w, this.x, this.y, this.z);
    }
}
