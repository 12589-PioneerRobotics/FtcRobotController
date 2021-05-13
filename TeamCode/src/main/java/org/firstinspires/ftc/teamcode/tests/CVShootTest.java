package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.ActuationConstants;
import org.firstinspires.ftc.teamcode.core.CVShooting;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_REST;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_YEET;
import static org.firstinspires.ftc.teamcode.tests.KobeTest2.shootDelayMillis;

@TeleOp
public class CVShootTest extends OpMode {
    CVShooting frame;
    GamepadEventPS update;
    StandardMechanumDrive drive;
    Actuation actuation;
    int currentTargetIndex = 0;
    OpenCvWebcam webcam;
    boolean turning = false;
    Servo cameraServo;


    @Override
    public void init() {
        frame = new CVShooting(ActuationConstants.Target.values()[currentTargetIndex], telemetry);
        update = new GamepadEventPS(gamepad1);
        drive = new StandardMechanumDrive(hardwareMap);
        cameraServo = hardwareMap.servo.get("cameraServo");
        actuation = new Actuation(hardwareMap, drive, null, this);
        cameraServo.setPosition(0.80);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(frame);
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT));
    }

    @Override
    public void loop() {

        if (update.dPadLeft()) {
            if (currentTargetIndex != 0)
                currentTargetIndex -= 1;
        }
        if (update.dPadRight()) {
            if (currentTargetIndex != ActuationConstants.Target.values().length - 1)
                currentTargetIndex += 1;
        }

        if(ActuationConstants.Target.values()[currentTargetIndex] == ActuationConstants.Target.TOWER_GOAL) {
            actuation.preheatShooter(ActuationConstants.Target.TOWER_GOAL);
        }
        else {
            actuation.preheatShooter(ActuationConstants.Target.POWER_SHOT_LEFT);
        }

        if(update.share())
            actuation.killFlywheel();

        if(update.square())
            actuation.shootInPlace(1);

        if (update.cross()) {
            if(!frame.isAligned())
                turning = true;
            else {
                actuation.preheatShooter(-4.0);
                for (int i = 0; i < 3; i++) {
                    actuation.feedRing();
                }
            }
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        if (gamepad1.right_trigger > .5) actuation.suck();
        else if (gamepad1.left_trigger > .5) actuation.spitOut();
        else if (gamepad1.right_trigger < .5 && gamepad2.left_trigger < .5) actuation.stopIntake();

        if(turning) {
            drive.setWeightedDrivePower(new Pose2d(0, 0, frame.targetDist() > 0 ? -0.3 : 0.3));
            if(frame.isAligned()) {
                turning = false;
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                actuation.preheatShooter(-3.7);
                for (int i = 0; i < 3; i++) {
                    actuation.feedRing();
                }
            }
        }

        frame.setTarget(ActuationConstants.Target.values()[currentTargetIndex]);

        telemetry.addLine("Press x to turn, square to shoot");
        telemetry.addData("Current target", frame.getTarget().toString());
        telemetry.addData("# Contours", frame.getContourCount());
        telemetry.addLine(frame.isAligned() ? "Aligned" : "Not aligned");
        telemetry.update();
    }
}
