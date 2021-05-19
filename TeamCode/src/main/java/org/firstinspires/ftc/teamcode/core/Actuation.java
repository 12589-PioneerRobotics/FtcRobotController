package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.CAMERA_POS_AIM;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.CAMERA_POS_RINGS;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_REST;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_YEET;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.LEFT_STICK_DOWN;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.LEFT_STICK_UP;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.RIGHT_STICKS_DOWN;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.RIGHT_STICK_UP;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_LEFT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_MIDDLE;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_RIGHT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.TOWER_GOAL;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_ARM_DOWN;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_ARM_UP;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_GRAB;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.WOBBLE_RELEASE;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.shooterPIDF;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.SHOOT_LINE;
import static org.firstinspires.ftc.teamcode.tests.KobeTest2.shootDelayMillis;

public class Actuation {

    DcMotorEx shoot;
    DcMotor intake, backIntakeBelt;
    public Servo wobbleGrab, wobbleArm, feeder, cameraServo, leftStick, rightStick;
    HardwareMap hardwareMap;
    StandardMechanumDrive drive;
    LinearOpMode linearOpMode;
    OpMode opMode;
    RevColorSensorV3 colorsensor;
    boolean shot;
    public boolean turnShooting = false;

    public boolean shotRight = false;
    public boolean shotMiddle = false;
    public boolean shotLeft = false;

    public CVShooting frame;

    public OpenCvWebcam getWebcam() {
        return webcam;
    }

    OpenCvWebcam webcam;

    int rings = 0;
    double ringRuntime = 0;

    /**
     * For Autonomous initialization specifically.
     */
    public Actuation(LinearOpMode linearOpMode, StandardMechanumDrive drive) {
        this(linearOpMode.hardwareMap, drive, linearOpMode, null);
        this.linearOpMode = linearOpMode;
        frame = new CVShooting(linearOpMode.telemetry);
    }

    /**
     * For TeleOp initialization specifically.
     */
    public Actuation(HardwareMap hardwareMap, StandardMechanumDrive drive, LinearOpMode linearOpMode, OpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.drive = drive;
        this.linearOpMode = linearOpMode;
        this.opMode = opMode;

        if(linearOpMode == null)
            frame = new CVShooting(opMode.telemetry);
        else  frame = new CVShooting(linearOpMode.telemetry);

        if (hardwareMap.dcMotor.contains("intake")) {
            intake = hardwareMap.dcMotor.get("intake");
            intake.setMode(STOP_AND_RESET_ENCODER);
            intake.setMode(RUN_WITHOUT_ENCODER);
        }

        if(hardwareMap.dcMotor.contains("backIntakeBelt")) {
            backIntakeBelt = hardwareMap.dcMotor.get("backIntakeBelt");
            backIntakeBelt.setMode(STOP_AND_RESET_ENCODER);
            backIntakeBelt.setMode(RUN_WITHOUT_ENCODER);
        }

        if (hardwareMap.dcMotor.contains("shooter")) {
            shoot = hardwareMap.get(DcMotorEx.class, "shooter");
            shoot.setMode(STOP_AND_RESET_ENCODER);
            shoot.setMode(RUN_USING_ENCODER);
            shoot.setZeroPowerBehavior(BRAKE);
            shoot.setPIDFCoefficients(RUN_USING_ENCODER, shooterPIDF);
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
            colorsensor = hardwareMap.get( RevColorSensorV3.class, "colorSensor");
            colorsensor.enableLed(true);
        }

        if (hardwareMap.servo.contains("feeder")) {
            feeder = hardwareMap.servo.get("feeder");
            feeder.setPosition(FEEDER_REST);
        }
        shot = false;

        if(hardwareMap.servo.contains("cameraServo")) {
            cameraServo = hardwareMap.servo.get("cameraServo");
            cameraServo.setPosition(CAMERA_POS_RINGS);
        }
        if(hardwareMap.servo.contains("rightStick") && hardwareMap.servo.contains("leftStick")) {
            rightStick = hardwareMap.servo.get("rightStick");
            leftStick = hardwareMap.servo.get("leftStick");

            rightStick.setPosition(RIGHT_STICK_UP);
            leftStick.setPosition(LEFT_STICK_UP);
        }
    }

    private Telemetry telemetry() {
        if(linearOpMode != null)
            return linearOpMode.telemetry;
        else return opMode.telemetry;
    }

    public void sticksUp() {
        if(rightStick == null || leftStick == null) return;
        rightStick.setPosition(RIGHT_STICK_UP);
        leftStick.setPosition(LEFT_STICK_UP);
    }

    public void sticksDown() {
        if(rightStick == null || leftStick == null) return;
        rightStick.setPosition(RIGHT_STICKS_DOWN);
        leftStick.setPosition(LEFT_STICK_DOWN);
    }

    public void cvShootingInit() {
        if(linearOpMode == null)
            frame = new CVShooting(opMode.telemetry);
        else frame = new CVShooting(linearOpMode.telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(frame);
        try {
            webcam.openCameraDeviceAsync(() -> webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT));
        }
        catch (OpenCvCameraException e) {
            telemetry().addLine(e.toString());
        }
        cameraServoToAim();
    }

    public void cameraServoToRings() {
        if(cameraServo != null)
            cameraServo.setPosition(CAMERA_POS_RINGS);
    }

    void cameraServoToAim() {
        if(cameraServo != null)
            cameraServo.setPosition(CAMERA_POS_AIM);
    }


    // All Shooter Operations

    public void feedRing() {
        try {
            feeder.setPosition(FEEDER_REST);
            Thread.sleep(350);
            feeder.setPosition(FEEDER_YEET);
            Thread.sleep(200);
            feeder.setPosition(FEEDER_REST);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        /*try {
            feeder.setPosition(FEEDER_REST);
            while(abs(shoot.getVelocity(RADIANS) - preheatShooter(frame.getTarget())) > 0.3) {
                telemetry().addData("Diff", abs(shoot.getVelocity(RADIANS) - preheatShooter(frame.getTarget())));
            }
            feeder.setPosition(FEEDER_YEET);
            Thread.sleep(200);
            feeder.setPosition(FEEDER_REST);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/
    }

    public boolean checkRings() {
        if(colorsensor == null) return false;
        if(colorsensor.red() > 50 && (opMode.getRuntime() - ringRuntime) > 0.5) {
            ringRuntime = opMode.getRuntime();
            if(rings == 3) {
                spitOut();
                return true;
            }
            rings += 1;
            return true;
        }
        return false;
    }

    public int getRings() {
        return rings;
    }

    /**
     * Turns the shooter (or robot if necessary) to face the red goal, then fires.
     */
    @Deprecated
    public void shoot() {
        shoot(TOWER_GOAL);
    }

    /**
     * Turns the shooter (or robot if necessary) to face the given target, and fires.
     *  @param target position to fire at
     *
     */
    @Deprecated
    public void shoot(Target target) {
        shoot(target, 0.10);
    }

    @Deprecated
    public void shoot(Target target, double offset) {
        if (shoot == null || feeder == null) return;
        Pose2d pose = drive.getPoseEstimate();

        double destination = target.pos().minus(pose.vec()).angle();
        destination = destination > PI ? destination - 2 * PI : destination;


        if (linearOpMode == null) { // If we are in TeleOp
            if (pose.getX() > SHOOT_LINE - 9) {
                drive.followTrajectory(
                        drive.trajectoryBuilder(pose)
                                .lineToLinearHeading(new Pose2d(SHOOT_LINE - 9, pose.getY(), destination))
                                .build()
                );

                drive.turn(destination - ((pose.getHeading() > PI) ? pose.getHeading() - (2 * PI) : pose.getHeading()) - offset);

                try {
                    feeder.setPosition(FEEDER_YEET);
                    Thread.sleep(500);
                    feeder.setPosition(FEEDER_REST);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        } else {
            linearOpMode.telemetry.addData("Start angle", pose.getHeading());
            linearOpMode.telemetry.addData("Destination", destination);
            linearOpMode.telemetry.update();
            drive.turn(destination - ((pose.getHeading() > PI) ? pose.getHeading() - (2 * PI) : pose.getHeading()) - offset);

            feeder.setPosition(FEEDER_YEET);
            linearOpMode.sleep(500);
            feeder.setPosition(FEEDER_REST);
        }

        /*feedRing();
        shoot.setVelocity(target == TOWER_GOAL ? -4.0 : -3.9*//*calcInitialSpeed(target)*//*, AngleUnit.RADIANS);
        if (linearOpMode != null)
            linearOpMode.sleep(700); *///TODO: Find appropriate delay, enough to let the motor shoot.
        drive.update();
    }

    public void shootCV(Target target) {
        cameraServoToAim();
        Target previousTarget = frame.getTarget();

        frame.setTarget(target);

        // Need a delay here for some reason if the target changes. Doesn't change fast enough -_-
        if(previousTarget != target) {
            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if(!frame.isAligned()) {
            while(!frame.isAligned() && linearOpMode.opModeIsActive()) {
                drive.setWeightedDrivePower(new Pose2d(0,0, cvShootTurnSpeed()));
            }
            drive.setWeightedDrivePower(new Pose2d(0,0,0));
        }
        feedRing();
    }

    public boolean shootCVTeleOp(Target target) {
        return shootCVTeleOp(target, 1);
    }

    public boolean shootCVTeleOp(Target target, int times) {
        Target previousTarget = frame.getTarget();
        frame.setTarget(target);
        if(previousTarget != target) {
            try {
                Thread.sleep(800);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if (turnShooting) {
            drive.setWeightedDrivePower(new Pose2d(0, 0, cvShootTurnSpeed()));
            if (frame.isAligned()) {
                turnShooting = false;
                drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                for(int i = 0; i < times; i++) {
                    feedRing();
                }
                return true;
            }
        }
        return false;
    }

    private double cvShootTurnSpeed() {
        double turnSpeed = 0.17;
        if(frame.getTarget() == TOWER_GOAL)
            turnSpeed = 0.26;
        return frame.targetDist() > 0 ? -turnSpeed : turnSpeed;
    }

    public void shootCV(int times) {
        for(int i = 0; i < times; i++) {
            shootCV();
        }
    }

    public void shootCV() {
        shootCV(TOWER_GOAL);
    }

    public void shootInPlace(int times) {
        for(int i = 0; i < times; i++) {
            feedRing();
        }
    }

    public void powerShots() {
        shootCV(POWER_SHOT_RIGHT);
        shootCV(POWER_SHOT_MIDDLE);
        shootCV(POWER_SHOT_LEFT);
    }

    public void powerShotsTeleOp() {

        if(!shotRight) {
            if (shootCVTeleOp(POWER_SHOT_RIGHT)) {
                shotRight = true;
                turnShooting = true;
            }
        }

        if(shotRight && !shotMiddle) {
            if (shootCVTeleOp(POWER_SHOT_MIDDLE)) {
                shotMiddle = true;
                turnShooting = true;
            }
        }

        if(shotRight && shotMiddle && !shotLeft) {
            if (shootCVTeleOp(POWER_SHOT_LEFT)) {
                shotLeft = true;
            }
        }
    }

    public double preheatShooter(double velocity) {
        if (shoot != null) shoot.setVelocity(velocity, RADIANS);
        return velocity;
    }

    public double preheatShooter(Target target) {
        if (shoot != null)
            shoot.setVelocity(target == TOWER_GOAL ? -3.7 : -3.1, RADIANS);
        return target == TOWER_GOAL ? -3.7 : -3.1;
    }

    public void killFlywheel() {
        if (shoot != null) shoot.setPower(0);
    }

    // All Intake Operations

    public void suck() {
        if (intake == null || backIntakeBelt == null || rings == 3) return;
        intake.setPower(1);
        backIntakeBelt.setPower(-1);
    }

    public void stopIntake() {
        if (intake == null || backIntakeBelt == null) return;
        intake.setPower(0);
        backIntakeBelt.setPower(0);
    }

    public void spitOut() {
        if (intake == null | backIntakeBelt == null) return;
        intake.setPower(-1);
        backIntakeBelt.setPower(1);

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

    public void wobbleArmSlightlyUp() {
        if (wobbleArm != null)
            wobbleArm.setPosition(.6);
    }

    public void placeWobble() {
        if (wobbleArm != null && wobbleGrab != null) {
//            wobbleArmDown();
            wobbleArm.setPosition(0.6);
            linearOpMode.sleep(750);
            wobbleClawOpen();
            linearOpMode.sleep(750);
            wobbleArmUp();
        }
    }

    public boolean isWobbleArmUp() {
        return wobbleArm.getPosition() == WOBBLE_ARM_UP;
    }

    public boolean isWobbleClawOpen() {
        return wobbleGrab.getPosition() == WOBBLE_RELEASE;
    }

}
