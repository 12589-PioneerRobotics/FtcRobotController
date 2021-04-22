package org.firstinspires.ftc.teamcode.core;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2HLS;
import static org.opencv.imgproc.Imgproc.boundingRect;

public class CVShooting extends OpenCvPipeline {

    private final double centerX = 50;
    private final double rangeX = 40;
    private final Scalar red = new Scalar(255, 0, 0);
    private final Scalar green = new Scalar(0, 255, 0);
    private final Scalar blue = new Scalar(0, 0, 255);
    private final Scalar orange = new Scalar(255, 165, 0);
    private double targetCenter = 9999999;

    Telemetry telemetry;

    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    private boolean isAligned;

    public boolean isAligned() {
        return isAligned;
    }

    public ActuationConstants.Target getTarget() {
        return target;
    }

    public void setTarget(ActuationConstants.Target target) {
        this.target = target;
    }

    ActuationConstants.Target target;

    public CVShooting(ActuationConstants.Target target, Telemetry telemetry) {
        this.target = target;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
//        Rect cropBox = new Rect(new Point(0,0), new Point(100,100));
//        Mat cropped = new Mat(input, cropBox);
        Mat hls = new Mat();
        Imgproc.cvtColor(input, hls, COLOR_RGB2HLS);

        int height = input.height();
        int width = input.width();

        // Shooting bounds
        Imgproc.line(input, new Point(centerX + rangeX,0), new Point(centerX + rangeX, height), orange);
        Imgproc.line(input, new Point(centerX - rangeX,0), new Point(centerX - rangeX, height), orange);

        Mat contourSrc = new Mat();
//        Core.inRange(hls, new Scalar(28, 50, 50), new Scalar(45, 255, 255), contourSrc);

        // Finding all red pixels in the source image
        Mat bw1 = new Mat();
        Mat bw2 = new Mat();
        Core.inRange(hls, new Scalar(0, 15,40), new Scalar(12, 255, 255), bw1);
        Core.inRange(hls, new Scalar(170, 70, 50), new Scalar(180, 255, 255), bw2);
        Core.add(bw1, bw2, contourSrc);

        Mat hierarchy = new Mat();
        Imgproc.findContours(contourSrc, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() == 3 || contours.size() == 4) {
            int largestContourIndex = 0;
            ArrayList<Rect> boundingRects = new ArrayList<>();
            ArrayList<Double> centers = new ArrayList<>();

            for (int i = 0; i < contours.size(); i++) {
                // This will prob work to get the tower goal
                if(i > 0) {
                    if (Imgproc.contourArea(contours.get(i)) > Imgproc.contourArea(contours.get(largestContourIndex))) {
                        largestContourIndex = i;
                    }
                }
                boundingRects.add(boundingRect(new MatOfPoint2f(contours.get(i).toArray())));
                centers.add(centerRect(boundingRects.get(i)));
            }
            Rect largestBoundingRect = boundingRect(new MatOfPoint2f(contours.get(largestContourIndex).toArray()));
            Imgproc.rectangle(input, largestBoundingRect.br(), largestBoundingRect.tl(), blue, 2);
            Imgproc.drawContours(input, contours, -1, green);
            Imgproc.drawContours(input, contours, largestContourIndex, blue);

            // Big assumption here: In order for power shots to be effective, all of them need to be in view of the camera.
            switch (target) {
                // With the above assumption made, the left power shot should always be with the lowest center x value
                case POWER_SHOT_LEFT:
                    targetCenter = Collections.min(centers);
                    break;

                case POWER_SHOT_MIDDLE:
                    centers.sort(Double::compareTo);
                    if(contours.size() == 3 || contours.size() == 4)
                        targetCenter = centers.get(1);
                    break;

                case POWER_SHOT_RIGHT:
                    centers.sort(Double::compareTo);
                    if(contours.size() == 3 || contours.size() == 4)
                        targetCenter = centers.get(2);
                    break;

                case TOWER_GOAL:
                    targetCenter = centerRect(largestBoundingRect);
                    break;
            }
        }
        else if (!contours.isEmpty()){
            Imgproc.drawContours(input, contours, -1, green);
            telemetry.addLine("Incorrect # objects detected");
            telemetry.update();
        }
        else {
            telemetry.addLine("Incorrect # objects detected");
            telemetry.update();

        }

        return input;
    }

    public int getContourCount() {
        return contours.size();
    }

    /**
     *
     * @return distance from centerX, in pixels. Right is positive, left is negative.
     */
    public double targetDist() {
        return targetCenter - centerX;
    }

    private double centerRect(Rect rect) {
        return (rect.br().x + rect.tl().x) / 2.0;
    }
}
