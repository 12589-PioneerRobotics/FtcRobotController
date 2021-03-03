package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_REST;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_YEET;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FLYWHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.POWER_SHOT_FIRE_VERTICAL_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.TOWER_GOAL_VERTICAL_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_LEFT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_MIDDLE;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_RIGHT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.TOWER_GOAL;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_ARM_DOWN;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_ARM_UP;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_GRAB;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_RELEASE;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.SHOOT_LINE;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.centerPowerShot;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.leftPowerShot;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.redGoal;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.rightPowerShot;

public class Actuation {

    DcMotorEx shoot;
    DcMotor intake;
    Servo wobbleGrab, wobbleArm, feeder;
    HardwareMap hardwareMap;
    Localizer localizer;
    LinearOpMode linearOpMode;
    RevColorSensorV3 colorsensor;
    boolean shot;

    /**
     * For Autonomous initialization specifically.
     */
    public Actuation(LinearOpMode linearOpMode, Localizer localizer) {
        this(linearOpMode.hardwareMap, localizer, linearOpMode);
        this.linearOpMode = linearOpMode;
    }

    /**
     * For TeleOp initialization specifically.
     */
    public Actuation(HardwareMap hardwareMap, Localizer localizer, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.localizer = localizer;
        this.linearOpMode = linearOpMode;

        if (hardwareMap.dcMotor.contains("intake")) {
            intake = hardwareMap.dcMotor.get("intake");
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (hardwareMap.dcMotor.contains("shooter")) {
            shoot = hardwareMap.get(DcMotorEx.class, "shooter");
            shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoot.setVelocityPIDFCoefficients(1, 1, 0, 1);
        }

        if (hardwareMap.servo.contains("wobbleGrab")) {
            wobbleGrab = hardwareMap.servo.get("wobbleGrab");
            if (linearOpMode != null)
                wobbleClawClose();
            else
                wobbleClawOpen();
        }

        if (hardwareMap.servo.contains("wobbleArm")) {
            wobbleArm = hardwareMap.servo.get("wobbleArm");
            wobbleArmUp();
        }

        if (hardwareMap.colorSensor.contains("colorSensor")) {
            colorsensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        }

        if (hardwareMap.servo.contains("feeder")) {
            feeder = hardwareMap.servo.get("feeder");
            feeder.setPosition(FEEDER_REST);
        }
        shot = false;
    }


    // All Shooter Operations

    public void feedRing() {
        if(shot) {
            feeder.setPosition(FEEDER_REST);
            shot = false;

        }
        else {
            feeder.setPosition(FEEDER_YEET);
            shot = true;
        }
    }

    public boolean hasRings() {
        if (colorsensor == null) return false;
        return Math.abs(colorsensor.getNormalizedColors().red - 170) > 20;
    }

    /**
     * Turns the shooter (or robot if necessary) to face the red goal, then fires.
     */
    public void shoot(StandardMechanumDrive drive) {
        shoot(TOWER_GOAL, drive);
    }

    /**
     * Turns the shooter (or robot if necessary) to face the given target, and fires.
     *
     *
     * @param target position to fire at
     * @param drive  driving instance to turn robot if necessary (>150 degrees)
     */
    public void shoot(Target target, StandardMechanumDrive drive) {
        if (shoot == null) return;
        Pose2d pose = localizer.getPoseEstimate();

        double destination = target.pos().minus(pose.vec()).angle();


        if (linearOpMode == null) { // If we are in TeleOp
            if (pose.getX() > SHOOT_LINE - 9) {
                drive.followTrajectory(
                        drive.trajectoryBuilder(pose)
                                .lineToLinearHeading(new Pose2d(SHOOT_LINE - 9, pose.getY(), destination))
                                .build()
                );
            }
        } else {
            linearOpMode.sleep(200);
            linearOpMode.telemetry.addData("Start angle", pose.getHeading());
            linearOpMode.telemetry.addData("Destination", destination);
            linearOpMode.telemetry.update();
            drive.turn(destination - pose.getHeading());
        }

        feedRing();
        shoot.setVelocity(target == TOWER_GOAL ? -4.0 : -3.9/*calcInitialSpeed(target)*/, AngleUnit.RADIANS);
        if (linearOpMode != null)
            linearOpMode.sleep(700); //TODO: Find appropriate delay, enough to let the motor shoot.
        drive.update();
    }

    public void powerShots(StandardMechanumDrive drive) {
        shoot(POWER_SHOT_LEFT, drive);
        shoot(POWER_SHOT_MIDDLE, drive);
        shoot(POWER_SHOT_RIGHT, drive);
    }

    public void preheatShooter(Target target) {
        if (shoot != null)
            shoot.setVelocity(target == TOWER_GOAL ? -4.0 : -3.9, AngleUnit.RADIANS);
    }

    public void killFlywheel() {
        if (shoot != null)
            shoot.setPower(0);
    }

    /**
     * Using a kinematics based approach to calculate the correct initial velocity to launch the ring so
     * it can reach its goal, based on the vertical/horizontal distance from the target. See
     * https://www.desmos.com/calculator/qxdrswohm7 for more details, and the engineering notebook
     * for a complete derivation.
     *
     * @param target One of 4 targets to shoot at (3 power shots, and the tower goal)
     * @return Adjusted speed to run the motor at (see adjust() for more details on the offset).
     */
    @Deprecated
    double calcInitialSpeed(Target target) {
        double h = 0.0; // in meters
        final double g = 9.8; // In m/s^2
        double d = 0.0; // in meters
        switch (target) {
            case TOWER_GOAL:
                h = TOWER_GOAL_VERTICAL_DISPLACEMENT;
                d = localizer.getPoseEstimate().vec().distTo(redGoal);
                break;
            case POWER_SHOT_LEFT:
                h = POWER_SHOT_FIRE_VERTICAL_DISPLACEMENT;
                d = localizer.getPoseEstimate().vec().distTo(leftPowerShot);
                break;
            case POWER_SHOT_RIGHT:
                h = POWER_SHOT_FIRE_VERTICAL_DISPLACEMENT;
                d = localizer.getPoseEstimate().vec().distTo(rightPowerShot);
                break;
            case POWER_SHOT_MIDDLE:
                h = POWER_SHOT_FIRE_VERTICAL_DISPLACEMENT;
                d = localizer.getPoseEstimate().vec().distTo(centerPowerShot);
                break;
        }
        double linearSpeed = sqrt((pow(d, 2) * g) / (pow(cos(LAUNCHER_ANGLE), 2) * (2 * d * tan(LAUNCHER_ANGLE) - 2 * h)));
        return adjust(linearSpeed / FLYWHEEL_RADIUS);
    }

    /**
     * Offsets the calculated, "theoretical" initial angular velocity (in rad/s) based on testing.
     * When the launcher actually is fed a ring, its set velocity (which is accurate to about +/- .5
     * rad/s) decreases due to the resistance from the ring. So, based on some tests we ran, the
     * speed is offsetted by about .75 rad/s to counter the decrease to keep expectations
     * consistent.
     *
     * @param angVel Speed from calcInitialSpeed()
     * @return Adjusted speed, still in rad/s
     */
    @Deprecated
    double adjust(double angVel) {
        return angVel + .75;
    }


    // All Intake Operations

    public void suck() {
        if (intake != null)
            intake.setPower(1);
    }

    public void stopIntake() {
        if (intake != null)
            intake.setPower(0);
    }

    public void spitOut() {
        if (intake != null)
            intake.setPower(-1);
    }


    // All Wobble Operations

    public void wobbleClawClose() {
        if (wobbleGrab != null)
            wobbleGrab.setPosition(WOBBLE_GRAB);
    }

    public void wobbleClawOpen() {
        if (wobbleGrab != null)
            wobbleGrab.setPosition(WOBBLE_RELEASE);
    }

    public void wobbleArmDown() {
        if (wobbleArm != null)
            wobbleArm.setPosition(WOBBLE_ARM_DOWN);
    }

    public void wobbleArmUp() {
        if (wobbleArm != null)
            wobbleArm.setPosition(WOBBLE_ARM_UP);
    }

    public void wobbleArmSlightltyUp() {
        if(wobbleArm != null)
            wobbleArm.setPosition(.6);
    }

    public void placeWobble() {
        if (wobbleArm != null && wobbleGrab != null) {
            wobbleArmDown();
            linearOpMode.sleep(750);
            wobbleClawOpen();
            linearOpMode.sleep(750);
            wobbleArmUp();
        }
    }

    public void grabWobble() {
        if (wobbleArm != null && wobbleGrab != null) {
            wobbleArmDown();
            linearOpMode.sleep(750);
            wobbleClawClose();
            linearOpMode.sleep(750);
            wobbleArmUp();
        }
    }

    public boolean isWobbleArmUp() {
        return wobbleArm.getPosition() == WOBBLE_ARM_UP;
    }

    public boolean isWobbleArmDown() {
        return wobbleArm.getPosition() == WOBBLE_ARM_DOWN;
    }

    public boolean isWobbleClawOpen() {
        return wobbleGrab.getPosition() == WOBBLE_RELEASE;
    }

    public boolean isWobbleClawClosed() {
        return wobbleGrab.getPosition() == WOBBLE_GRAB;
    }

}
