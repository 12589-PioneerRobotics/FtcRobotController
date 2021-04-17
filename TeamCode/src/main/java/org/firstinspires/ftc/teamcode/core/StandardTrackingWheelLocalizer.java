package org.firstinspires.ftc.teamcode.core;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    // Remember, all applicable measurements are in inches.
    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = .984;
    public static double GEAR_RATIO = 1;

    public static double LATERAL_DISTANCE = 15.9; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6.5; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public static Pose2d leftDeadWheelPose = new Pose2d(-2.25, LATERAL_DISTANCE / 2, 0);
    public static Pose2d rightDeadWheelPose = new Pose2d(-3.25, -LATERAL_DISTANCE / 2, 0);
    public static Pose2d horizontalDeadWheelPose = new Pose2d(2.0, 8.0, Math.toRadians(90));

    public static double VERTICAL_MULTIPLIER = 1.0026;
    public static double HORIZONTAL_MULTIPLIER = 0.9885;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                leftDeadWheelPose,
                rightDeadWheelPose,
                horizontalDeadWheelPose
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backIntakeBelt"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "sideways_deadwheel"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * VERTICAL_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * VERTICAL_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * HORIZONTAL_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()) * VERTICAL_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getRawVelocity()) * VERTICAL_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * HORIZONTAL_MULTIPLIER
        );
    }
}
