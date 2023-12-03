package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PropPipeline extends OpenCvPipeline {
    private Init.Side side; //side==Init.Side.BOARD means the centre prop is to the right of image, aka board auto
    private double bound = 80;
    public String ans="none";
    private Scalar lower, upper, lower2, upper2;
    public PropPipeline(Init.Team team, Init.Side side) {
        if (team == Init.Team.RED) {
            lower = new Scalar(127,50,144);
            upper = new Scalar(180,112,255);
            lower2 = new Scalar(0,50,52);
            upper2 = new Scalar(15,150,206);
        } else if (team == Init.Team.BLUE) {
            lower = new Scalar(74,50,62);
            upper = new Scalar(109,115,255);
            lower2 = lower; upper2 = upper;
        }
        this.side=side;

    }
    @Override
    public Mat processFrame(Mat input) {
        // convert to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, org.opencv.imgproc.Imgproc.COLOR_RGB2HSV);
        //inrange for colour use lower and upper
        Mat mask = new Mat(), mask2 = new Mat();
        Core.inRange(hsv, lower, upper, mask);
        Core.inRange(hsv, lower2, upper2, mask2);
        Core.bitwise_or(mask, mask2, mask);
        //get image width and height
        int width = input.width();
        int height = input.height();

        Rect left = new Rect(0, 200, (int)bound, 190);
        Rect centre = new Rect(side== Init.Side.BOARD ? 280 : 170, 200, side==Init.Side.BOARD ? 470-280 : 360-170, 190);
        Rect right = new Rect((int)(width-bound), 200, (int)bound, 190);
        Mat leftMat = new Mat(mask, left);
        Mat centreMat = new Mat(mask, centre);
        Mat rightMat = new Mat(mask, right);

        Imgproc.line(input, new org.opencv.core.Point(bound, 0), new org.opencv.core.Point(bound, mask.height()), new Scalar(0,0,255), 2);
        Imgproc.line(input, new org.opencv.core.Point(width-bound, 0), new org.opencv.core.Point(width-bound, mask.height()), new Scalar(0,0,255), 2);
        Imgproc.line(input, new org.opencv.core.Point(0, 200), new org.opencv.core.Point(mask.width(), 200), new Scalar(0,0,255), 2);
        Imgproc.line(input, new org.opencv.core.Point(0, 390), new org.opencv.core.Point(mask.width(), 390), new Scalar(0,0,255), 2);
        Imgproc.line(input, new org.opencv.core.Point(side==Init.Side.BOARD ? 280 : 170, 200), new org.opencv.core.Point(side==Init.Side.BOARD ? 280 : 170, 390), new Scalar(255,0,0), 2);
        Imgproc.line(input, new org.opencv.core.Point(side==Init.Side.BOARD ? 470 : 360, 200), new org.opencv.core.Point(side==Init.Side.BOARD ? 470 : 360, 390), new Scalar(255,0,0), 2);

        List<MatOfPoint> contours_left = new ArrayList<>(), contours_centre = new ArrayList<>(), contours_right = new ArrayList<>();
        Imgproc.findContours(leftMat, contours_left, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(centreMat, contours_centre, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(rightMat, contours_right, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint l = findProp(contours_left, 300);
        MatOfPoint c = findProp(contours_centre, 6400);
        MatOfPoint r = findProp(contours_right, 300);
        // offset l, c, and r by their respective rects
        if (l != null) {
            Rect lRect = Imgproc.boundingRect(l);
            lRect.x += left.x;
            lRect.y += left.y;
            Imgproc.rectangle(input, lRect, new Scalar(0,255,0), 2);
        }
        if (c != null) {
            Rect cRect = Imgproc.boundingRect(c);
            cRect.x += centre.x;
            cRect.y += centre.y;
            Imgproc.rectangle(input, cRect, new Scalar(0,255,0), 2);
        }
        if (r != null) {
            Rect rRect = Imgproc.boundingRect(r);
            rRect.x += right.x;
            rRect.y += right.y;
            Imgproc.rectangle(input, rRect, new Scalar(0,255,0), 2);
        }
        String defaultt = side==Init.Side.BOARD ? "right" : "left"; //sounds weird bc reversed
        ans = l!=null? "left" : c!=null ? "centre" : r!=null ? "right" : defaultt;
        
        return input;

    }
    private MatOfPoint findProp(List<MatOfPoint> contours, int area) {
        if (contours.size() > 0) {
            MatOfPoint a = Collections.max(contours, (o1, o2) -> (int) (Imgproc.contourArea(o1) - Imgproc.contourArea(o2)));
            if (Imgproc.contourArea(a) < area) {
                return null;
            } else {
                return a;
            }
        } else {
            return null;
        }
    }
}
