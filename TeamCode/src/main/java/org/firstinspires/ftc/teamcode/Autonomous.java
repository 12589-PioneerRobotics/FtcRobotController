package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.TensorFlowRingDetection;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.centerA;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.centerB;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.centerC;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.startPose;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {

    /*
        Routine:
        1. Perform CV Operations. Will tell us which case we need to pursue for the rest of autonomous.
        2. Shoot preloaded rings at power shots.
        3. Collect rings (if applicable).
        4. Shoot rings into top most goal (if applicable).
        4. Go to corresponding square, drop wobble goals in said squares.
        5. Park.
     */

    private TensorFlowRingDetection ringDetection;
    private String ringCase = "";
    private StandardMechanumDrive drive;
    Actuation actuation;

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new StandardMechanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        actuation = new Actuation(this, drive.getLocalizer());

//        ringDetection = new TensorFlowRingDetection(this);

        waitForStart();
        ringCase = "Quad"; //ringDetection.res(this); //TODO: Hardcoding for now. Change when camera is mounted
        telemetry.addData("Ring case", ringCase);
        telemetry.update();

        if (isStopRequested()) return;
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(6).build());
        actuation.powerShots(drive);
        performCase(ringCase);
        park();
        hardwareMap.appContext
                .getSharedPreferences("Auton end pose", Context.MODE_PRIVATE).edit()
                .putString("serialized", drive.getPoseEstimate().toString())
                .apply();
    }

    void park() {
        Pose2d pose = drive.getPoseEstimate();
        drive.followTrajectory(drive.trajectoryBuilder(pose).lineToConstantHeading(new Vector2d(SHOOT_LINE, pose.getY())).build());
    }

    /**
        1. Go to center square of desired square
        2. Release 1st wobble (already preloaded)
        3. Go to start area
        4. Grab other wobble
        5. Go back to same case square
        6. Drop off 2nd wobble
     */
    void wobbleRoutine(Vector2d center) {
        // centerPose is a pose of the square's center (A/B/C), backPose is the position the robot will be in to collect the second wobble goal
        Pose2d centerPose = new Pose2d(center.getX(), center.getY(), 0);
        Pose2d backPose = new Pose2d(-55,-48, toRadians(-90));

        // Go to square for 1st time, drop off preloaded wobble
        drive.followTrajectory(drive.staticSpline(center).build());
        actuation.placeWobble();

        // Go back to start area to get 2nd wobble, go back to same square
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate(), toRadians(90))
                .splineToSplineHeading(backPose, toRadians(180))
                .build()
        );

        // Collect 2nd wobble (right side), go back to drop off second wobble and place it
        actuation.grabWobble();
        drive.followTrajectory(
                drive.trajectoryBuilder(backPose, toRadians(0))
                .splineToSplineHeading(centerPose, toRadians(0)).build());
        actuation.placeWobble();
    }

    private void performCase(String ringCase) {
        Vector2d ringPos = new Vector2d(-24, -36);
        Trajectory startToRings = drive.trajectoryBuilder(startPose).splineToConstantHeading(ringPos, 0).build();
        switch (ringCase) {
            case "None": // Zero rings, case "A"
                wobbleRoutine(centerA);
                break;

            case TensorFlowRingDetection.LABEL_SECOND_ELEMENT: // One ring, case "B", "Single"
                actuation.suck();
                drive.followTrajectory(startToRings);
                actuation.stopIntake();
                actuation.shoot(drive); //TODO: Account for loading rings into shooter for case B, C
                wobbleRoutine(centerB);
                break;

            case TensorFlowRingDetection.LABEL_FIRST_ELEMENT: // 4 rings, case "C", "Quad"
                actuation.suck();
                drive.followTrajectory(startToRings);
                actuation.stopIntake();

                actuation.shoot(drive);
                actuation.shoot(drive);
                actuation.shoot(drive);

                wobbleRoutine(centerC);
                break;
        }
    }
}
