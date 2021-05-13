package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.ActuationConstants;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;
import org.openftc.easyopencv.OpenCvWebcam;

import static java.lang.Double.parseDouble;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_REST;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_YEET;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_LEFT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_MIDDLE;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_RIGHT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.TOWER_GOAL;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.autonStartPose;
import static org.firstinspires.ftc.teamcode.tests.KobeTest2.shootDelayMillis;

/*
    Controls:
    Gamepad1:
    -Movement
    -Shooting

    Gamepad2:
    - intake (Right Trigger)
    - Wobble grab / place (Triangle)

 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    StandardMechanumDrive drive;
    Actuation actuation;
    GamepadEventPS update1, update2;
    boolean normalslowMode = false;
    boolean reverseSlowMode = false;
    boolean targettingTowerGoal = false; // Shooter is either set to target tower goal or powershot

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
//        update2 = new GamepadEventPS(gamepad2);
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
            if(actuation.isWobbleArmUp())
                actuation.wobbleArmDown();
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
            if(targettingTowerGoal) {
                for (int i = 0; i < 3; i++) {
                    actuation.feedRing();
                }
            }
            else {
                actuation.powerShots();
            }
        }

        if(update1.share())
            actuation.killFlywheel();

        /* Uncomment for localization debugging
        telemetry.addData("x", drive.getPoseEstimate().getX());
        telemetry.addData("y", drive.getPoseEstimate().getY());
        telemetry.addData("heading", drive.getPoseEstimate().getHeading());*/
        telemetry.addData("Rings" ,actuation.getRings());
        telemetry.addData("Inverse slow mode", reverseSlowMode? "On" : "Off");
        telemetry.addData("Slow Mode", normalslowMode ? "On" : "Off");
        telemetry.addData("Targeting", targettingTowerGoal ? "Tower" : "Power Shots");
        telemetry.update();
        drive.update();
    }

    /**
     * Between autonomous and teleop, we want to be able to "quicksave" our location at the end of
     * autonomous and teleop. This is primarily for knowing where we are on the field so we can
     * shoot automatically without driver guidance. We do this by adding a serialized version of our
     * last Pose2d instance to UserPreferences (a small scale implementation of storage on Android),
     * and retrieving it and "unserializing it" manually. //TODO: Fix this
     *
     * @param s toString() of last known Pose2d instance
     * @return Pose2d instance
     */
    static Pose2d unserialize(String s) {
        String[] components = s.substring(1, s.length() - 1).split(",");
        double x = parseDouble(components[0].trim());
        double y = parseDouble(components[1].trim());
        double heading = parseDouble(components[2].trim());
        return new Pose2d(x, y, heading);
    }

    public static double powerScale(double power){
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
