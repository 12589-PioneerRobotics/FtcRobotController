package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.CVShooting;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_LEFT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.TOWER_GOAL;

/**
 * Controls:
 * (Only one gamepad)
 *
 * Triangle: Move sticks up down
 *
 * Right Trigger: Intake In
 * Left Trigger: Intake Out
 *
 * Left Joystick: Translational movement
 * Left Joystick Click: Toggle slow mode
 * D-Pad Up: Reverse slow mode
 * Right Joystick: Rotational movement
 *
 * Square: Change target (Powershots or Tower goal)
 * Circle: Automated shooting (All 3 powershots or 3 shots into tower goal)
 * Cross: Manual single shot
 * Share: Kill flywheel
 *
 * Left Bumper: Wobble claw open/close
 * Right Bumper: Wobble arm up/down
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    StandardMechanumDrive drive;
    Actuation actuation;
    GamepadEventPS update1, update2;
    boolean normalslowMode = false;
    boolean reverseSlowMode = false;
    boolean targettingTowerGoal = false; // Shooter is either set to target tower goal or powershot
    boolean sticksUp = false;

    @Override
    public void init() {
        drive = new StandardMechanumDrive(hardwareMap);
        Pose2d startPose;
        String serialized = hardwareMap.appContext.getSharedPreferences("Auton end pose", Context.MODE_PRIVATE)
                .getString("serialized", "");

        SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences("Auton end pose", Context.MODE_PRIVATE);
        startPose = new Pose2d(prefs.getLong("x", 12), prefs.getLong("y", 0), prefs.getLong("heading", (long)toRadians(-90)));
        drive.setPoseEstimate(startPose);
        actuation = new Actuation(hardwareMap, drive, null, this);
        update1 = new GamepadEventPS(gamepad1);
//        actuation.preheatShooter(TOWER_GOAL);
        actuation.cvShootingInit();

    }

    @Override
    public void loop() {

        // Translational movement
        if(update1.leftStickButton()) {
            if(reverseSlowMode) reverseSlowMode = false;
            normalslowMode = !normalslowMode;
        }


        if(normalslowMode) {
            drive.setDrivePower(
                    new Pose2d(
                            powerScale(gamepad1.left_stick_y, .35),
                            powerScale(gamepad1.left_stick_x, .35),
                            -powerScale(gamepad1.right_stick_x, .35)
                    )
            );
        }
        else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
        }

        // Intake functionality
        if (gamepad1.right_trigger > .5) actuation.suck();
        else if (gamepad1.left_trigger > .5) actuation.spitOut();
        else if (gamepad1.right_trigger < .5 && gamepad2.left_trigger < .5) actuation.stopIntake();

        // Wobble grabber/arm functionality (left/right bumpers)
        if(update1.rightBumper()) {
            if(actuation.isWobbleArmUp()) {
                actuation.wobbleArmDown();
//                actuation.wobbleClawClose();
            }
            else actuation.wobbleArmUp();
        }

        if(update1.leftBumper()) {
            if(actuation.isWobbleClawOpen())
                actuation.wobbleClawClose();
            else actuation.wobbleClawOpen();
        }

        /*if (actuation.hasRings()) {
            actuation.preheatShooter();
            telemetry.addLine("Rings present");
        } else actuation.killFlywheel();*/
        
        // Inverse + Slow
        if(gamepad1.dpad_up) {
            reverseSlowMode = true;
            drive.setWeightedDrivePower(new Pose2d(-0.35, 0, 0));
        }

        if(reverseSlowMode) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -powerScale(gamepad1.left_stick_y, .35),
                            -powerScale(gamepad1.left_stick_x, .35),
                            -powerScale(gamepad1.right_stick_x, .35)
                    )
            );
        }


        if(gamepad1.dpad_left)
            drive.setWeightedDrivePower(new Pose2d(0, -0.2, 0));

        if(gamepad1.dpad_right)
            drive.setWeightedDrivePower(new Pose2d(0, 0.2, 0));


        if(update1.square()) {
            targettingTowerGoal = !targettingTowerGoal;
            if(targettingTowerGoal)
                actuation.preheatShooter(TOWER_GOAL);
            else actuation.preheatShooter(POWER_SHOT_LEFT);
        }

        if(update1.cross())
            actuation.feedRing();

        if(update1.circle()) {
            actuation.turnShooting = true;
            actuation.shotLeft = false;
            actuation.shotRight = false;
            actuation.shotMiddle = true;
        }

        if(update1.triangle()) {
            sticksUp = !sticksUp;
        }

        if(sticksUp)
            actuation.sticksDown();
        else actuation.sticksUp();

        if(targettingTowerGoal)
            actuation.shootCVTeleOp(TOWER_GOAL, 3);
        else actuation.powerShotsTeleOp();

        if(update1.share())
            actuation.killFlywheel();

        if(update1.ps()) {
            actuation.frame = new CVShooting(telemetry);
            actuation.cvShootingInit();
        }

        // Uncomment for localization debugging
        /*telemetry.addData("x", drive.getPoseEstimate().getX());
        telemetry.addData("y", drive.getPoseEstimate().getY());
        telemetry.addData("heading", drive.getPoseEstimate().getHeading());*/
        telemetry.addData("Rings" ,actuation.getRings());
        telemetry.addData("Inverse slow mode", reverseSlowMode? "On" : "Off");
        telemetry.addData("Slow Mode", normalslowMode ? "On" : "Off");
        telemetry.addData("Targeting", targettingTowerGoal ? "Tower" : "Power Shots");
        telemetry.update();
        drive.update();
    }

    public static double powerScale(double power) {
        return powerScale(power, 1);
    }

    public static double powerScale(double power, double scale){
        if (power<=1) {
            if (power < 0)
                return -(scale * power * power);
            else
                return scale * power * power;
        }
        return 1;
    }
}
