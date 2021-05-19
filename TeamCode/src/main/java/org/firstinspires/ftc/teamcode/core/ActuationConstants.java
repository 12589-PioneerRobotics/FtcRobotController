package org.firstinspires.ftc.teamcode.core;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.*;

@Config
public class ActuationConstants {

    // Wobble Grabber Constants
    //TODO: Measure Wobble Constants.
    static final double WOBBLE_GRAB = 0.90;
    static final double WOBBLE_RELEASE = 0.0;

    static final double WOBBLE_ARM_UP = 0.0;
    static final double WOBBLE_ARM_DOWN = 1.0;

    // Sticks
    static final double LEFT_STICK_UP = 0.575;
    static final double LEFT_STICK_DOWN = 0.9;
    static final double RIGHT_STICK_UP = 0.575;
    static final double RIGHT_STICKS_DOWN = 0.225;

    // Launcher Constants
    public static final double FLYWHEEL_RADIUS = toMeters(2);
    public static final double LAUNCHER_ANGLE = toRadians(32);
    static final double LAUNCHER_HEIGHT = toMeters(9); // Measure where center of mass of disk would be before being launched.

    public static PIDFCoefficients shooterPIDF = new PIDFCoefficients(30,25,0,0);

    static final double CAMERA_POS_RINGS = 0.65;
    static final double CAMERA_POS_AIM = 0.8;

    public static double FEEDER_REST = 0.375;
    public static double FEEDER_YEET = 0.61;

    //TODO: Get a better measurement of power shot heights, considering the slant.
    static final double POWER_SHOT_FIRE_VERTICAL_DISPLACEMENT = toMeters(23.5) - LAUNCHER_HEIGHT;
    public static final double TOWER_GOAL_VERTICAL_DISPLACEMENT = toMeters(35.5) - LAUNCHER_HEIGHT;

    public enum Target {
        POWER_SHOT_LEFT,
        POWER_SHOT_MIDDLE,
        POWER_SHOT_RIGHT,
        TOWER_GOAL;

        public Vector2d pos() {
            switch (this) {
                case POWER_SHOT_LEFT:
                    return leftPowerShot;
                case POWER_SHOT_MIDDLE:
                    return centerPowerShot;
                case POWER_SHOT_RIGHT:
                    return rightPowerShot;
                case TOWER_GOAL:
                    return redGoal;
                default:
                    return new Vector2d();
            }
        }

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case TOWER_GOAL:
                    return "Tower Goal";
                case POWER_SHOT_LEFT:
                    return "Left Power Shot";
                case POWER_SHOT_RIGHT:
                    return "Right Power Shot";
                case POWER_SHOT_MIDDLE:
                    return "Middle Power Shot";
                default:
                    return "???";
            }
        }
    }

    /**
     * Converts inches to meters
     * @param in Length, in inches
     * @return Length, in meters
     */
    public static double toMeters(double in) { return in / 39.3700787; }
}
