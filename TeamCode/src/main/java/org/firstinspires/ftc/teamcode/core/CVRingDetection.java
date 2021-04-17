package org.firstinspires.ftc.teamcode.core;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class CVRingDetection extends OpenCvPipeline {
    private String ringCase;
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private int boundingRectHeight = 0;

    @Override
    public Mat processFrame(Mat input) {
        Rect cropBox = new Rect(new Point(0,0), new Point(100,100));
        Mat cropped = new Mat(input, cropBox);
        Imgproc.cvtColor(input, cropped, Imgproc.COLOR_RGB2HLS);

        Mat contourSrc = new Mat();
        Core.inRange(cropped, new Scalar(28, 50, 50), new Scalar(45, 255, 255), contourSrc);
//        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(contourSrc, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        if(contours.isEmpty()) {
            ringCase = "None";
        }
        else {
            int largestContourIndex = 0;
            for (int i = 1; i < contours.size(); i++) {
                if (Imgproc.contourArea(contours.get(i)) > Imgproc.contourArea(contours.get(largestContourIndex)))
                    largestContourIndex = i;
            }
            Rect boundingRectangle = Imgproc.boundingRect(new MatOfPoint2f(contours.get(largestContourIndex).toArray()));
            Imgproc.rectangle(input, boundingRectangle.br(), boundingRectangle.tl(), new Scalar(0,255,0), 2);
            boundingRectHeight = boundingRectangle.height;
            Imgproc.drawContours(input, contours, largestContourIndex, new Scalar(0,255,0));

        }

        return input;
    }

    public String getRingCase() {
        return ringCase;
    }
    public int getContourCount() {
        return contours.size();
    }
    public int getBoundingRectHeight() {
        return boundingRectHeight;
    }
}
