package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.TensorFlowRingDetection;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

import static java.lang.Math.exp;
import static java.lang.Math.subtractExact;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_RIGHT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.TOWER_GOAL;
import static org.firstinspires.ftc.teamcode.core.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.core.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.core.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.core.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.centerA;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.centerB;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.centerC;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.autonStartPose;
import static org.firstinspires.ftc.teamcode.core.TensorFlowRingDetection.LABEL_FIRST_ELEMENT;
import static org.firstinspires.ftc.teamcode.core.TensorFlowRingDetection.LABEL_SECOND_ELEMENT;

@Autonomous(name = "Autonomous")
public class AutonomousRemote extends LinearOpMode {

    /*
        Routine:
        1. Perform CV Operations. Will tell us which case we need to pursue for the rest of autonomous.
        2. Shoot preloaded rings at power shots.
        3. Collect rings (if applicable).
        4. Shoot rings into top most goal (if applicable).
        4. Go to corresponding square, drop wobble goals in said squares.
        5. Park.
     */

    TensorFlowRingDetection ringDetection;
    String ringCase = "";
    StandardMechanumDrive drive;
    Actuation actuation;

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new StandardMechanumDrive(hardwareMap);
        drive.setPoseEstimate(autonStartPose);
        actuation = new Actuation(this, drive);
        ringDetection = new TensorFlowRingDetection(this);

        Thread cvThread = new Thread(ringDetection);
        cvThread.start();

        waitForStart();

        ringCase = ringDetection.ringCase;
        ringDetection.stopFlag = true;
        telemetry.addData("Ring case here", ringCase);
        telemetry.update();


        if (isStopRequested()) return;

        performCase(ringCase);
        park();
        hardwareMap.appContext
                .getSharedPreferences("Auton end pose", Context.MODE_PRIVATE).edit()
                .putString("serialized", drive.getPoseEstimate().toString())
                .putLong("x", (long)drive.getPoseEstimate().getX())
                .putLong("y", (long)drive.getPoseEstimate().getY())
                .putLong("heading", (long)drive.getPoseEstimate().getHeading())
                .apply();
    }

    void park() {
        Pose2d pose = drive.getPoseEstimate();
        drive.followTrajectory(drive.trajectoryBuilder(pose).lineToConstantHeading(new Vector2d(SHOOT_LINE, pose.getY())).build());
    }

    /**
     * 1. Go to center square of desired square
     * 2. Release 1st wobble (already preloaded)
     * 3. Go to start area
     * 4. Grab other wobble
     * 5. Go back to same case square
     * 6. Drop off 2nd wobble
     */
    void wobbleRoutine(Pose2d center, Pose2d back) {
        // centerPose is a pose of the square's center (A/B/C), backPose is the position the robot will be in to collect the second wobble goal
        Pose2d centerAgain = center;
        centerAgain = new Pose2d(centerAgain.getX() - 10, centerAgain.getY() + 5, centerAgain.getHeading());

        if(center.epsilonEquals(centerC))
            SHOOT_LINE = 8;

        // Go to square for 1st time, drop off preloaded wobble
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(center, toRadians(-90))
                        .addTemporalMarker(1, () -> actuation.wobbleArmDown())
                        .build()
        );

        actuation.wobbleClawOpen();
        sleep(550);
        actuation.wobbleArmUp();

        // Go back to start area to get 2nd wobble, go back to same square
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate(), toRadians(180))
                        .splineToLinearHeading(back, toRadians(180))
//                        .splineTo(back.vec(), toRadians(180))
                        .build()
        );

        // Collect 2nd wobble (right side), go back to drop off second wobble and place it
        actuation.wobbleArmDown();
        sleep(1000);
        actuation.wobbleClawClose();
        sleep(750);
        actuation.wobbleArmSlightltyUp();

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate(), drive.getPoseEstimate().getHeading())
                        .splineToLinearHeading(centerAgain, toRadians(180))
                        .build()
        );
        actuation.placeWobble();
    }

    public void experimentalWobbleGoalRoutine(Pose2d center, Pose2d back) {
        // centerPose is a pose of the square's center (A/B/C), backPose is the position the robot will be in to collect the second wobble goal
        Pose2d centerAgain = center;
        centerAgain = new Pose2d(centerAgain.getX() - 10, centerAgain.getY() + 5, centerAgain.getHeading());

        double limiter = 0.5;
        MinVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL * limiter),
                new MecanumVelocityConstraint(MAX_VEL * limiter, TRACK_WIDTH)
        ));
        ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL * limiter);

        telemetry.addLine("Calculating trajectory...");
        telemetry.update();
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate(), velConstraint, accelConstraint, toRadians(-90))
                .splineToSplineHeading(center, toRadians(-90))
                .addTemporalMarker(1, () -> actuation.wobbleArmDown())
                .addSpatialMarker(center.vec(), () -> {
                    actuation.wobbleClawOpen();
//                    sleep(500);
//                    actuation.wobbleArmDown();
                })

                .splineToSplineHeading(back, toRadians(130))
                .addSpatialMarker(back.vec(), () -> actuation.wobbleClawClose())

                .splineToSplineHeading(centerAgain, toRadians(180))
                .addSpatialMarker(centerAgain.vec(), () -> actuation.wobbleClawOpen())
                .build();

        telemetry.addLine("Done!");
        telemetry.update();

        // Go to square for 1st time, drop off preloaded wobble
        drive.followTrajectory(trajectory);

        actuation.placeWobble();

/*        actuation.wobbleClawOpen();
        sleep(550);
        actuation.wobbleArmUp();

        // Go back to start area to get 2nd wobble, go back to same square
*//*        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate(), toRadians(180))
                        .splineToLinearHeading(back, toRadians(180))
//                        .splineTo(back.vec(), toRadians(180))
                        .build()
        );*//*

        // Collect 2nd wobble (right side), go back to drop off second wobble and place it
        actuation.wobbleArmDown();
        sleep(1000);
        actuation.wobbleClawClose();
        sleep(750);
        actuation.wobbleArmSlightltyUp();

*//*        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate(), drive.getPoseEstimate().getHeading())
                        .splineToLinearHeading(centerAgain, toRadians(180))
                        .build()
        );*//*
        actuation.placeWobble();*/
    }

    private void performCase(String ringCase) {
        Trajectory startToRings;
        switch (ringCase) {
            case "None": // Zero rings, case "A"
                actuation.preheatShooter(-3.88);

//                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).splineToConstantHeading(ringPos, 0).build());
                startToRings = drive.trajectoryBuilder(drive.getPoseEstimate()).splineToLinearHeading(new Pose2d(0,-30), 0).build();
                drive.followTrajectory(startToRings);

                actuation.shoot(TOWER_GOAL, 0.1);

                sleep(300);


                actuation.shootInPlace(2);

                actuation.killFlywheel();
//                wobbleRoutine(centerA, backPoseA);
                experimentalWobbleGoalRoutine(centerA, backPoseA);
                break;

            case LABEL_SECOND_ELEMENT: // One ring, case "B", "Single"
                actuation.preheatShooter(-3.9);
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(12).build());

                actuation.shoot(TOWER_GOAL, 0.17);

//                actuation.preheatShooter(TOWER_GOAL);
                actuation.suck();
                startToRings = drive.trajectoryBuilder(drive.getPoseEstimate()).splineToLinearHeading(new Pose2d(0,-30), 0).build();
                drive.followTrajectory(startToRings);

                telemetry.addData("current pose", drive.getPoseEstimate().toString());

                actuation.shoot(TOWER_GOAL, 0.15);
                actuation.stopIntake();
                sleep(300);
                actuation.suck();

                actuation.shootInPlace(2);

                actuation.stopIntake();
                actuation.killFlywheel();
//                wobbleRoutine(centerB, backPoseB);
                experimentalWobbleGoalRoutine(centerB, backPoseB);
                break;

            case LABEL_FIRST_ELEMENT: // 4 rings, case "C", "Quad"

                actuation.preheatShooter(-3.91);
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(12).build());
                sleep(400);
                actuation.shoot(TOWER_GOAL, 0.17);
                sleep(400);


                actuation.shootInPlace(2);

                actuation.preheatShooter(POWER_SHOT_RIGHT);
                actuation.suck();

                startToRings = drive.trajectoryBuilder(drive.getPoseEstimate()).splineToConstantHeading(ringPos, 0).build();
                drive.followTrajectory(startToRings);
                actuation.stopIntake();

                actuation.preheatShooter(-3.8);

                //actuation.powerShotsC(0.14,0.08,0.06);
                actuation.shoot(TOWER_GOAL, 0.15);


                actuation.shootInPlace(1);

                actuation.stopIntake();
                actuation.killFlywheel();

//                wobbleRoutine(centerC, backPoseC);
                experimentalWobbleGoalRoutine(centerC, backPoseC);
                break;
        }
    }
}
