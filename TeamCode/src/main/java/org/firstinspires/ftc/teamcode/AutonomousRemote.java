package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.TensorFlowRingDetection;

import java.util.Arrays;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_LEFT;
import static org.firstinspires.ftc.teamcode.core.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.core.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.core.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.core.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.SHOOT_LINE;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.autonStartPose;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.backPoseA;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.backPoseB;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.backPoseC;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.centerA;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.centerB;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.centerC;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.leftOfRingPos;
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
    StandardMechanumDrive drive;
    Actuation actuation;

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new StandardMechanumDrive(hardwareMap);
        drive.setPoseEstimate(autonStartPose);
        actuation = new Actuation(this, drive);

        actuation.cameraServoToRings();
        ringDetection = new TensorFlowRingDetection(this, drive);

        new Thread(ringDetection).start();
        waitForStart();
        ringDetection.stopFlag = true;

        if (isStopRequested()) return;

        try {
            performCase(ringDetection);
        } catch (ExecutionException e) {
            e.printStackTrace();
        }
        park();
        hardwareMap.appContext
                .getSharedPreferences("Auton end pose", Context.MODE_PRIVATE).edit()
                .putString("serialized", drive.getPoseEstimate().toString())
                .putLong("x", (long) drive.getPoseEstimate().getX())
                .putLong("y", (long) drive.getPoseEstimate().getY())
                .putLong("heading", (long) drive.getPoseEstimate().getHeading())
                .apply();
    }

    void park() {
        Pose2d pose = drive.getPoseEstimate();
        drive.followTrajectory(drive.trajectoryBuilder(pose).lineToConstantHeading(new Vector2d(SHOOT_LINE, pose.getY())).build());
    }    Future<Trajectory> startToRingsTask;


    /**
     * 1. Go to center square of desired square 2. Release 1st wobble (already preloaded) 3. Go to
     * start area 4. Grab other wobble 5. Go back to same case square 6. Drop off 2nd wobble
     */
    void wobbleRoutine(Pose2d center, Pose2d back) throws ExecutionException, InterruptedException {
        // centerPose is a pose of the square's center (A/B/C), backPose is the position the robot will be in to collect the second wobble goal
        Pose2d centerAgain = center;
        centerAgain = new Pose2d(centerAgain.getX() - 10, centerAgain.getY() + 5, centerAgain.getHeading());

        if (center.epsilonEquals(centerC))
            SHOOT_LINE = 8;

        // Go to square for 1st time, drop off preloaded wobble
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(center, toRadians(-90))
                        .addTemporalMarker(1, () -> actuation.wobbleArmDown())
                        .build()
        );

        Future<Trajectory> centerToBackTask = drive.trajectory(() ->
                drive.trajectoryBuilder(drive.getPoseEstimate(), toRadians(180))
                .splineToLinearHeading(back, toRadians(180))
                .build());

        actuation.wobbleClawOpen();
        sleep(550);
        actuation.wobbleArmUp();

        // Go back to start area to get 2nd wobble, go back to same square
        drive.followTrajectory(centerToBackTask.get());

        Pose2d finalCenterAgain = centerAgain;
        Future<Trajectory> backToCenterTask = drive.trajectory(() ->
                drive.trajectoryBuilder(drive.getPoseEstimate(), drive.getPoseEstimate().getHeading())
                        .splineToLinearHeading(finalCenterAgain, toRadians(180))
                        .build());

        // Collect 2nd wobble (right side), go back to drop off second wobble and place it
        actuation.wobbleArmDown();
        sleep(1000);
        actuation.wobbleClawClose();
        sleep(750);
        actuation.wobbleArmSlightlyUp();

        Trajectory backToCenter = backToCenterTask.get();
        drive.followTrajectory(backToCenter);
        actuation.placeWobble();
    }

    public void experimentalWobbleGoalRoutine(Pose2d center, Pose2d back) {
        // centerPose is a pose of the square's center (A/B/C), backPose is the position the robot will be in to collect the second wobble goal
        Pose2d centerAgain = center;
        centerAgain = new Pose2d(centerAgain.getX() - 7, centerAgain.getY() + 5, centerAgain.getHeading());

        double limiter = 0.5;
        MinVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL * limiter),
                new MecanumVelocityConstraint(MAX_VEL * limiter, TRACK_WIDTH)
        ));
        ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL * limiter);

        Pose2d backEntry = new Pose2d(back.getX() + 10, back.getY() - 5);
        Pose2d backExit = new Pose2d(back.getX() + 10, back.getY() + 5);

        telemetry.addLine("Calculating trajectory...");
        telemetry.update();
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate(), velConstraint, accelConstraint, toRadians(-90))
                .splineToSplineHeading(center, toRadians(-90))
                .addTemporalMarker(0.5, () -> actuation.wobbleArmDown())
                .addSpatialMarker(center.vec(), () -> actuation.wobbleClawOpen())

                .splineToSplineHeading(back, toRadians(130))
                .addSpatialMarker(back.vec(), () -> {
                    actuation.wobbleClawClose();
                    sleep(500);
                    actuation.wobbleArmSlightlyUp();
                })

                .splineToSplineHeading(centerAgain, toRadians(180))
                .addSpatialMarker(centerAgain.vec(), () -> actuation.wobbleClawOpen())
                .build();

        telemetry.addLine("Done!");
        telemetry.update();

        // Go to square for 1st time, drop off preloaded wobble
        drive.followTrajectory(trajectory);

        actuation.placeWobble();
    }

    private void performCase(TensorFlowRingDetection thread) throws ExecutionException, InterruptedException {
        Future<Trajectory> startToRings;
        switch (thread.ringCase) {
            case "None": // Zero rings, case "A"
                actuation.preheatShooter(POWER_SHOT_LEFT);

                drive.followTrajectory(drive.trajectoryBuilder(
                        drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(0, -30), 0)
                        .build());

                actuation.powerShots();
                actuation.killFlywheel();

//                wobbleRoutine(centerA, backPoseA);
                experimentalWobbleGoalRoutine(centerA, backPoseA);
                break;

            case LABEL_SECOND_ELEMENT: // One ring, case "B", "Single"
                actuation.preheatShooter(POWER_SHOT_LEFT);
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(12).build());

//                actuation.shoot(TOWER_GOAL, 0.17);
                actuation.powerShots();

                actuation.suck();
                drive.followTrajectory(drive.trajectoryBuilder(
                        drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(0, -30), 0)
                        .build());

                telemetry.addData("current pose", drive.getPoseEstimate().toString());

                actuation.shootCV();
                actuation.stopIntake();
                sleep(300);
                actuation.suck();

                actuation.stopIntake();
                actuation.killFlywheel();
                wobbleRoutine(centerB, backPoseB);
//                experimentalWobbleGoalRoutine(centerB, backPoseB);
                break;

            case LABEL_FIRST_ELEMENT: // 4 rings, case "C", "Quad"

                actuation.preheatShooter(POWER_SHOT_LEFT);
                /*drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(12).build());
                sleep(400);
                actuation.shoot(TOWER_GOAL, 0.17);
                sleep(400);*/

//                startToRings = drive.trajectoryBuilder(drive.getPoseEstimate()).splineToConstantHeading(ringPos, 0).build();

                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate(), toRadians(45))
                        .splineToConstantHeading(leftOfRingPos, toRadians(225))
                        .splineToConstantHeading(new Vector2d(-2, -30), toRadians(135))
                        .build());

                actuation.suck();
                actuation.powerShots();


                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).splineToSplineHeading(new Pose2d(-16, -30, 0), toRadians(0)).build());
                actuation.shootCV(2);

                telemetry.addLine("First Round finished");
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).splineToSplineHeading(new Pose2d(-22, -30, 0), toRadians(0)).build());
                telemetry.addLine("Trajectory finished");
                actuation.shootCV(2);
                actuation.stopIntake();

                actuation.killFlywheel();

                wobbleRoutine(centerC, backPoseC);
//                experimentalWobbleGoalRoutine(centerC, backPoseC);
                break;
        }
    }
}
