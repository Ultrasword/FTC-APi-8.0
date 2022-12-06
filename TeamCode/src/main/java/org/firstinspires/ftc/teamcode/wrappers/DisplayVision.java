package org.firstinspires.ftc.teamcode.wrappers;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class DisplayVision extends OpenCvPipeline {
    private static final Scalar
            lower_yellow = new Scalar(20,140,140),
            upper_yellow = new Scalar(30,255,255),
            lower_cyan = new Scalar(95,100,100),
            upper_cyan = new Scalar(105,255,255),
            lower_magenta = new Scalar(160,100,100),
            upper_magenta = new Scalar(170,255,255);
    private Mat hsv = new Mat(), yellow = new Mat(), cyan = new Mat(), magenta = new Mat(), mask = new Mat(), hierarchy = new Mat();
    private List<MatOfPoint> contours = new java.util.ArrayList<>();
    public int route = 0;
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lower_yellow, upper_yellow, yellow);
        Core.inRange(hsv, lower_cyan, upper_cyan, cyan);
        Core.inRange(hsv, lower_magenta, upper_magenta, magenta);
        Core.bitwise_or(yellow, cyan, mask);
        Core.bitwise_or(mask, magenta, mask);
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() > 0) {
            double maxArea = 0;
            int maxAreaIdx = -1;
            for (int idx = 0; idx < contours.size(); idx++) {
                Mat contour = contours.get(idx);
                double contourArea = Imgproc.contourArea(contour);
                if (contourArea > maxArea) {
                    maxArea = contourArea;
                    maxAreaIdx = idx;
                }
            }
            if (maxArea<50) return input;
            double avgHue = 0;
            for (Point point : contours.get(maxAreaIdx).toArray()) {
                double[] hsv = input.get((int)point.y, (int)point.x);
                avgHue += hsv[0];
            }
            avgHue /= contours.get(maxAreaIdx).toArray().length;
            Rect rect = Imgproc.boundingRect(contours.get(maxAreaIdx));
            if (avgHue >= 20 && avgHue <= 30) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 255, 0), 2);
                route = 0;
            } else if (avgHue >= 95 && avgHue <= 105) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 255), 2);
                route = 1;
            } else if (avgHue >= 160 && avgHue <= 170) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 0, 255), 2);
                route = 2;
            }
        }
        contours.clear();
        return input;
    }
}
