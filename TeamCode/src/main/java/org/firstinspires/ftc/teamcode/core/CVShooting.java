package org.firstinspires.ftc.teamcode.core;


import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.drawMarker;

public class CVShooting extends OpenCvPipeline {

    private double centerX = 200;
    private final double rangeX = 15;
    private final Scalar red = new Scalar(255, 0, 0);
    private final Scalar green = new Scalar(0, 255, 0);

    private final Scalar blue = new Scalar(0, 0, 255);
    private final Scalar orange = new Scalar(255, 165, 0);
    private double targetCenter = 9999999;
    private final double offsetAngle = Math.toRadians(10);

    Telemetry telemetry;

    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    public boolean isAligned() {
        return (targetCenter < centerX + rangeX && targetCenter > centerX - rangeX);
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
        hsv = new Mat();
        bw1 = new Mat();
        bw2 = new Mat();
        contourSrc = new Mat();
        hierarchy = new Mat();
        kernelSize = new Size(3,3);
        redLb1 = new Scalar(0, 150, 0);
        redUb1 = new Scalar(12, 255, 255);
        redLb2 = new Scalar(170, 90, 0);
        redUb2 = new Scalar(180, 255, 255);
    }
    Mat hsv, bw1, bw2, contourSrc, hierarchy, cropped;
    Rect cropBox;
    ArrayList<Rect> boundingRects;
    ArrayList<Double> centers;
    Rect largestBoundingRect;
    Size kernelSize;
    int width, height;
    Scalar redLb1, redUb1, redLb2, redUb2;


    @Override
    public Mat processFrame(Mat input) {

        height = input.height();
        width = input.width();

        cropBox = new Rect(new Point(0,60), new Point(width,height));

        Imgproc.drawMarker(input, new Point(0,100), orange ,0);
        drawMarker(input, new Point(width,height), orange, 0);

        hsv = new Mat(input, cropBox);
        cropped = new Mat(input, cropBox);

        Imgproc.cvtColor(cropped, hsv, COLOR_RGB2HSV);
        Imgproc.GaussianBlur(hsv, hsv, kernelSize, 0);

        // Shooting bounds
        Imgproc.line(cropped, new Point(centerX + rangeX,0), new Point(centerX + rangeX, height), orange, 3);
        Imgproc.line(cropped, new Point(centerX - rangeX,0), new Point(centerX - rangeX, height), orange, 3);


        // Finding all red pixels in the source image
        Core.inRange(hsv, redLb1, redUb1, bw1);
        Core.inRange(hsv, redLb2, redUb2, bw2);
        Core.add(bw1, bw2, contourSrc);

        contours = new ArrayList<>();
        Imgproc.findContours(contourSrc, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if(!contours.isEmpty()) contours.removeIf(a -> contourArea(a) < 20);


        if (!contours.isEmpty()) {
            int largestContourIndex = 0;
            boundingRects = new ArrayList<>();
            centers = new ArrayList<>();


            largestContourIndex = getLargestContourIndex(contours);
            for (int i = 0; i < contours.size(); i++) {
                boundingRects.add(boundingRect(new MatOfPoint2f(contours.get(i).toArray())));
                centers.add(centerRectX(boundingRects.get(i)));
            }
            largestBoundingRect = boundingRect(new MatOfPoint2f(contours.get(largestContourIndex).toArray()));



            contours.removeIf(a -> {
                double centerLargestX = centerRectX(largestBoundingRect);
                double centerContourX = centerRectX(boundingRect(a));
                double centerLargestY = centerRectY(largestBoundingRect);
                double centerContourY = centerRectY(boundingRect(a));
                double halfWidth = largestBoundingRect.width / 2.0;
                double halfHeight = largestBoundingRect.height / 2.0;
                return ((centerLargestX + halfWidth) > centerContourX && (centerLargestX - halfWidth) < centerContourX) || (centerLargestY > centerContourY);
            });

            largestContourIndex = getLargestContourIndex(contours);

            /*int finalLargestContourIndex = largestContourIndex;
            contours.removeIf(a -> {
               Rect boundingRect = boundingRect(a);
               return boundingRect.width > boundingRect.height && contours.indexOf(a) != finalLargestContourIndex;
            });
            if(contours.isEmpty()) return input;*/

            /*for (int i = 0; i < contours.size(); i++) {
                // This will prob work to get the tower goal
                if(i > 0) {
                    if (Imgproc.contourArea(contours.get(i)) > Imgproc.contourArea(contours.get(largestContourIndex))) {
                        largestContourIndex = i;
                    }
                }
                boundingRects.add(boundingRect(new MatOfPoint2f(contours.get(i).toArray())));
                centers.add(centerRect(boundingRects.get(i)));
            }*/

            Imgproc.rectangle(cropped, largestBoundingRect.br(), largestBoundingRect.tl(), blue, 2);
            if(!contours.isEmpty()) {
                largestContourIndex = getLargestContourIndex(contours);
                Imgproc.drawContours(cropped, contours, -1, green);
                Log.v("largest contour index", String.valueOf(largestContourIndex));
                Imgproc.drawContours(cropped, contours, largestContourIndex, blue);


                // Big assumption here: In order for power shots to be effective, all of them need to be in view of the camera.
                switch (target) {
                    // With the above assumption made, the left power shot should always be with the lowest center x value
                    case POWER_SHOT_LEFT:
                        targetCenter = Collections.min(centers);
                        break;

                    case POWER_SHOT_MIDDLE:
                        centers.sort(Double::compareTo);
                        if (contours.size() == 3 || contours.size() == 4)
                            targetCenter = centers.get(1);
                        break;

                    case POWER_SHOT_RIGHT:
                        centers.sort(Double::compareTo);
                        if (contours.size() == 3 || contours.size() == 4)
                            targetCenter = centers.get(2);
                        break;

                    case TOWER_GOAL:
                        targetCenter = centerRectX(largestBoundingRect);
                        break;
                }
            }
        }
        else {
//            Imgproc.drawContours(input, contours, -1, green);
            telemetry.addLine("Incorrect # objects detected");
            telemetry.update();
        }

        return cropped;
    }

    int getLargestContourIndex(ArrayList<MatOfPoint> contourList) {
        int largest = 0;
        for (int i = 0; i < contourList.size(); i++) {
            // This will prob work to get the tower goal
            if (i > 0) {
                if (Imgproc.contourArea(contourList.get(i)) > Imgproc.contourArea(contourList.get(largest))) {
                    largest = i;
                }
            }
        }
        return largest;
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

    private double centerRectX(Rect rect) {
        return (rect.br().x + rect.tl().x) / 2.0;
    }

    private double centerRectY(Rect rect) {
        return (rect.br().y + rect.tl().y) / 2.0;
    }

}
