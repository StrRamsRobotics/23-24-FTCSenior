package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PropPipeline extends OpenCvPipeline {
    public double x, y;
    private Scalar lower, upper;
    public PropPipeline(Init.Team team) {
        if (team == Init.Team.RED) {
            lower = new Scalar(127,0,144);
            upper = new Scalar(180,112,255);
        } else if (team == Init.Team.BLUE) {
            lower = new Scalar(74,0,62);
            upper = new Scalar(109,115,255);
        }
    }
    @Override
    public Mat processFrame(Mat input) {
        // convert to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, org.opencv.imgproc.Imgproc.COLOR_RGB2HSV);
        //inrange for colour use lower and upper
        Mat mask = new Mat();
        Core.inRange(hsv, lower, upper, mask);
        //find contours using retr_tree
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) {

            MatOfPoint maxContour = contours.get(0);
            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > Imgproc.contourArea(maxContour)) {
                    maxContour = contour;
                }
            }
            //find bounding rect
            Rect rect = Imgproc.boundingRect(maxContour);
            //draw rectangle
            Imgproc.rectangle(input, rect, new Scalar(0, 255, 0), 2);
            //get centre coord of rectangle
            double x = rect.x + rect.width/2.0;
            double y = rect.y + rect.height/2.0;
            this.x=x; this.y=y;
        }
        return input;

    }
}
