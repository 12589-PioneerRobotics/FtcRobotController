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

    public static double LATERAL_DISTANCE = 15.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6.5; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public static Pose2d leftDeadWheelPose = new Pose2d(-2.25, LATERAL_DISTANCE / 2, 0);
    public static Pose2d rightDeadWheelPose = new Pose2d(-3.25, -LATERAL_DISTANCE / 2, 0);
    public static Pose2d horizontalDeadWheelPose = new Pose2d(2.0, LATERAL_DISTANCE / 2, Math.toRadians(90));

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                leftDeadWheelPose,
                rightDeadWheelPose,
                horizontalDeadWheelPose
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));

        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder.setDirection(Encoder.Direction.FORWARD);
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()),
                encoderTicksToInches(rightEncoder.getRawVelocity()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }
}
