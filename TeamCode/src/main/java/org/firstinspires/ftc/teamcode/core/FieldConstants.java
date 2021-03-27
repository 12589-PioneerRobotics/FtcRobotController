package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import static java.lang.Math.toRadians;

@Config
public class FieldConstants {
    public static double MAX_EDGE = 72;
    public static double SHOOT_LINE = 12; // Theoretical: 12

    // For shooting coordinates
    public static Vector2d redGoal = new Vector2d(MAX_EDGE, -44);
    public static Vector2d leftPowerShot = new Vector2d(MAX_EDGE, -4.5);
    public static Vector2d centerPowerShot = new Vector2d(MAX_EDGE, -9);
    public static Vector2d rightPowerShot = new Vector2d(MAX_EDGE, -13.5);

    // For path coordinates
    public static Vector2d ringPos = new Vector2d(0, -30);
    public static Pose2d centerA = new Pose2d(12,-44.5, toRadians(90)); // Theoretical: x = 12, y = -48
    public static Pose2d centerB = new Pose2d(32,-20, toRadians(90)); // Theoretical: x = 36, y = -24
    public static Pose2d centerC = new Pose2d(56, -44, toRadians(90)); // Theoretical: x = 60, y = -48

    // Theoretical: x = -31, y = -48
    public static Pose2d backPoseA = new Pose2d(-38,-32, toRadians(0)); // -49.25, -28
    public static Pose2d backPoseB = new Pose2d(-37, -34, toRadians(0)); // -47.5, -44
    public static Pose2d backPoseC = new Pose2d(-35,-30, toRadians(0)); // -52, -26

    public static Pose2d autonStartPose = new Pose2d(-63, -33, toRadians(0));
}
