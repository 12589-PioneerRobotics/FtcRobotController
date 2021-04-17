package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.ActuationConstants;
import org.firstinspires.ftc.teamcode.core.DriveConstants;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static org.firstinspires.ftc.teamcode.TeleOp.powerScale;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_REST;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_YEET;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.LAUNCHER_ANGLE;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.TOWER_GOAL_VERTICAL_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_LEFT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_MIDDLE;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_RIGHT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.shooterPIDF;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.toMeters;
import static org.firstinspires.ftc.teamcode.core.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.autonStartPose;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.leftPowerShot;

@Config
@TeleOp(name = "Kobe Test Part 2")
public class KobeTest2 extends OpMode {
    DcMotorEx shooter;
    Servo feeder;
    final double wheelRadius = 0.0508; // in meters
    public static double angularVelocity = 0; // rad/s
    public static double shootDelayMillis = 500;
    double[] turnOffsets = {0,0,0};
    public static double turnOffsetLeft = 0.18;
    public static double turnOffsetMiddle = .18;
    public static double turnOffsetRight = 0.18;

    double feederPosIncrement = .01;
    double feederPos = 0;


    double linearVelocity = 0; // m/s

    final double MAX_RPM = 6000.0; // in RPM, if you couldn't figure it out
    final double MAX_RAD = MAX_RPM * ((2.0 * PI) / 60.0);
    final double MAX_LINEAR_SPEED = MAX_RAD * wheelRadius; // m/s

    double distance = 73;

    StandardMechanumDrive drive;
    GamepadEventPS update;
    FtcDashboard dashboardTelemetry;


    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        feeder = hardwareMap.servo.get("feeder");
        feeder.setPosition(FEEDER_REST);
        update = new GamepadEventPS(gamepad1);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive = new StandardMechanumDrive(this.hardwareMap);
        drive.setPoseEstimate(autonStartPose);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        dashboardTelemetry = FtcDashboard.getInstance();
//        telemetry = dashboardTelemetry.getTelemetry();
    }

    @Override
    public void loop() {

        if(update.dPadUp())
            angularVelocity += .05;
        if(update.dPadDown())
            angularVelocity -= .05;

        if(gamepad1.left_bumper)
            feeder.setPosition(FEEDER_YEET);
        else
            feeder.setPosition(FEEDER_REST);

        if(update.dPadLeft())
            feederPos -= feederPosIncrement;
        if(update.dPadRight())
            feederPos += feederPosIncrement;

        drive.setDrivePower(
                new Pose2d(
                        powerScale(gamepad1.left_stick_y, .35),
                        powerScale(gamepad1.left_stick_x, .35),
                        -powerScale(gamepad1.right_stick_x, .35)
                )
        );

        linearVelocity = shooter.getVelocity(AngleUnit.RADIANS) * wheelRadius;

        turnOffsets = new double[]{turnOffsetRight, turnOffsetMiddle, turnOffsetLeft};
        // Shoot twice
        if(update.circle()) {
            ActuationConstants.Target[] targets = {POWER_SHOT_RIGHT, POWER_SHOT_MIDDLE, POWER_SHOT_LEFT};
            for (int i = 0; i < 3; i++) {
                double destination = targets[i].pos().minus(drive.getPoseEstimate().vec()).angle();
                telemetry.addData("destination", destination);
                Pose2d pose = drive.getPoseEstimate();
                drive.turn(destination - ((pose.getHeading() > PI) ? pose.getHeading() - (2 * PI) : pose.getHeading()) - turnOffsets[i]);
                feeder.setPosition(FEEDER_YEET);
                try {

                    Thread.sleep(200);
                    feeder.setPosition(FEEDER_REST);
                    Thread.sleep((long) Math.max(shootDelayMillis - 100, 0));
                    feeder.setPosition(FEEDER_YEET);

                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        }


        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        shooter.setVelocity(angularVelocity, AngleUnit.RADIANS);
        telemetry.addData("Velocity (m/s) ??", toMeters(encoderTicksToInches(shooter.getVelocity())));
        telemetry.addLine("Use dPad Up/Down to change motor speed");
        telemetry.addData("Set velocity (rad/s)", angularVelocity);
        telemetry.addData("Actual velocity (rad/s)", shooter.getVelocity(AngleUnit.RADIANS));
//        telemetry.addData("Feeder pos")

        /*telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));*/
        telemetry.update();
//        drive.update();
    }
    double calcInitialSpeed() {
        double h = TOWER_GOAL_VERTICAL_DISPLACEMENT; // in meters
        final double g = 9.8; // In m/s^2
        double d = distance; // in meters
        double linearSpeed = sqrt((pow(d, 2) * g) / (pow(cos(LAUNCHER_ANGLE), 2) * (2 * d * tan(LAUNCHER_ANGLE) - (2 * h) )));
        return linearSpeed;// / FLYWHEEL_RADIUS;
    }
}
