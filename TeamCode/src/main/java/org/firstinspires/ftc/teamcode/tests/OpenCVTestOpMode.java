package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.core.CVRingDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


@Autonomous(name = "OpenCV Test")
public class OpenCVTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpenCvCamera webcam;
        CVRingDetection frame = new CVRingDetection();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(frame);

        waitForStart();
        if(isStopRequested()) return;

        while(opModeIsActive()) {

            telemetry.addData("# Contours", frame.getContourCount());
            telemetry.addData("Ring case", frame.getRingCase());
            telemetry.addData("Bounding rect height", frame.getBoundingRectHeight());

            telemetry.update();
        }
    }
}
