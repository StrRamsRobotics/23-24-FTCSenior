package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class PropPipeline extends OpenCvPipeline {
    private Init.Side side; //side==Init.Side.BOARD means the centre prop is to the right of image, aka board auto
    private double bound = 80;
    public String ans="none";
    private Scalar lower, upper, lower2, upper2;
    public PropPipeline(Init.Team team, Init.Side side) {
        if (team == Init.Team.RED) {
            lower = new Scalar(127,30,144);
            upper = new Scalar(180,70,255);
            lower2 = new Scalar(0,30,100);
            upper2 = new Scalar(15,70,206);
        } else if (team == Init.Team.BLUE) {
            lower = new Scalar(82,25,30);
            upper = new Scalar(109,108,201); //originally 1115
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

        //erode with 3x3 kernel
//        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(3,3));
//        Imgproc.erode(mask, mask, kernel);
//        Imgproc.dilate(mask, mask, kernel);
        //get image width and height
        int width = input.width();
        int height = input.height();

        Rect left = new Rect(0, 190, (int)bound, 210);
        Rect centre = new Rect(side== Init.Side.BOARD ? 250 : 140, 190, side==Init.Side.BOARD ? 500-250 : 390-140, 210);
        Rect right = new Rect((int)(width-bound), 190, (int)bound, 210);
        Mat leftMat = new Mat(mask, left);
        Mat centreMat = new Mat(mask, centre);
        Mat rightMat = new Mat(mask, right);



        List<MatOfPoint> contours_left = new ArrayList<>(), contours_centre = new ArrayList<>(), contours_right = new ArrayList<>();
        Imgproc.findContours(leftMat, contours_left, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(centreMat, contours_centre, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(rightMat, contours_right, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint l = findProp(contours_left, 300); //originslly 300
        MatOfPoint c = findProp(contours_centre, 6400);
        MatOfPoint r = findProp(contours_right, 300);
        String defaultt = side==Init.Side.BOARD ? "right" : "left"; //sounds weird bc reversed
        MatOfPoint[] contours = {l,c,r};
        Arrays.sort(contours, (MatOfPoint a, MatOfPoint b)->{
            if (a==null&&b!=null) {
                return 1;
            } else if (a!=null&&b==null) {
                return -1;
            } else if (a==null&&b==null) {
                return 0;
            }
                return Integer.compare((int) Imgproc.contourArea(b), (int) Imgproc.contourArea(a));
        });
        if (contours[0]==null) {
            ans = defaultt;
        } else {
            ans = contours[0] == l ? "left" : contours[0] == c ? "centre" : contours[0] == r ? "right" : defaultt;
        }
        if (contours[0]!=null) {
            Rect rect = Imgproc.boundingRect(contours[0]);
            if (contours[0]==l) {
                rect.x += left.x; rect.y +=left.y;
            }
            if (contours[0]==c) {
                rect.x += centre.x; rect.y +=centre.y;
            }
            if (contours[0]==r) {
                rect.x += right.x; rect.y +=right.y;
            }
            Imgproc.rectangle(input, rect, new Scalar(0,255,0),2);
        }
        Imgproc.line(input, new Point(bound, 0), new Point(bound, input.height()), new Scalar(0,0,255), 2);
        Imgproc.line(input, new Point(width-bound, 0), new Point(width-bound, input.height()), new Scalar(0,0,255), 2);
        Imgproc.line(input, new Point(0, 190), new Point(input.width(), 190), new Scalar(0,0,255), 2);
        Imgproc.line(input, new Point(0, 400), new Point(input.width(), 400), new Scalar(0,0,255), 2);
        Imgproc.line(input, new Point(side==Init.Side.BOARD ? 250 : 140, 190), new Point(side==Init.Side.BOARD ? 250 : 140, 400), new Scalar(255,0,0), 2);
        Imgproc.line(input, new Point(side==Init.Side.BOARD ? 500 : 390, 190), new Point(side==Init.Side.BOARD ? 500 : 390, 400), new Scalar(255,0,0), 2);
// offset l, c, and r by their respective rects
//        if (l != null) {
//            if ((c!=null&&Imgproc.contourArea(l)>Imgproc.contourArea(c))||(r!=null&&Imgproc.contourArea(l)>Imgproc.contourArea(r))) {
//                ans = "left";
//            }
//            Rect lRect = Imgproc.boundingRect(l);
//            lRect.x += left.x;
//            lRect.y += left.y;
//            Imgproc.rectangle(input, lRect, new Scalar(0,255,0), 2);
//        }
//        if (c != null) {
//            if ((l!=null&&Imgproc.contourArea(c)>Imgproc.contourArea(l))||(r!=null&&Imgproc.contourArea(c)>Imgproc.contourArea(r))) {
//                ans = "centre";
//            }
//            Rect cRect = Imgproc.boundingRect(c);
//            cRect.x += centre.x;
//            cRect.y += centre.y;
//            Imgproc.rectangle(input, cRect, new Scalar(0,255,0), 2);
//        }
//        if (r != null) {
//            if ((c!=null&&Imgproc.contourArea(r)>Imgproc.contourArea(c))||(l!=null&&Imgproc.contourArea(r)>Imgproc.contourArea(l))) {
//                ans = "right";
//            }
//            Rect rRect = Imgproc.boundingRect(r);
//            rRect.x += right.x;
//            rRect.y += right.y;
//            Imgproc.rectangle(input, rRect, new Scalar(0,255,0), 2);
//        }
//        if (l==null&&c==null&&r==null) ans = defaultt;
        return mask;

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
