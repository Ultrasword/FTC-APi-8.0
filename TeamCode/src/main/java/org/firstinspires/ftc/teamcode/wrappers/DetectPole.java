package org.firstinspires.ftc.teamcode.wrappers;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class DetectPole extends OpenCvPipeline {
    private static final Scalar lower_yellow = new Scalar(15,130,130), upper_yellow = new Scalar(25,255,255);
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(1, 10));
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours = new java.util.ArrayList<>();
    private final double polePosition = 200;
    public double error=0;
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Core.inRange(input, lower_yellow, upper_yellow, input);
        Imgproc.morphologyEx(input, input, Imgproc.MORPH_OPEN, kernel);
        contours.clear();
        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() > 0) {
            double maxWidth = 0;
            int maxIndex = -1;
            for (int i = 0; i < contours.size(); i++) {
                Rect rect = Imgproc.boundingRect(contours.get(i));
                if (rect.width > maxWidth && rect.height > 4*rect.width) {
                    maxWidth = rect.width;
                    maxIndex = i;
                }
            }
            if (maxIndex != -1) {
                Rect maxRect = Imgproc.boundingRect(contours.get(maxIndex));
                error = maxRect.x + maxRect.width / 2 - polePosition;
            }
        }
        return input;
    }
}