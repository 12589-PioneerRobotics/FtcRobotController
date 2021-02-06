package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

import static java.lang.Double.parseDouble;

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
    boolean slowMode = false;

    @Override
    public void init() {
        drive = new StandardMechanumDrive(this.hardwareMap);
        Pose2d startPose;
        String serialized = ""; /*hardwareMap.appContext.getSharedPreferences("Auton end pose", Context.MODE_PRIVATE)
                .getString("serialized", "");*/
        if (serialized.equals(""))
            startPose = new Pose2d(-63, -36, 0);
        else startPose = unserialize(serialized);
        drive.setPoseEstimate(startPose);
        actuation = new Actuation(hardwareMap, drive.getLocalizer(), null);
        update1 = new GamepadEventPS(gamepad1);
        update2 = new GamepadEventPS(gamepad2);
    }

    @Override
    public void loop() {

        // Translational movement
        if(update1.leftStickButton()) {
            slowMode = !slowMode;
            if(slowMode) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                powerScale(gamepad1.left_stick_y),
                                powerScale(gamepad1.left_stick_x),
                                -powerScale(gamepad1.right_stick_x)
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
        }

        // Intake functionality
        if (gamepad2.right_trigger > .5) actuation.suck();
        if (gamepad2.left_trigger > .5) actuation.spitOut();
        if (gamepad2.right_trigger < .5 && gamepad2.left_trigger < .5) actuation.stopIntake();

        // Wobble grabber/arm functionality (triangle, square)
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

        /*if (update1.triangle()) {
            if (actuation.isWobbleArmUp()) {
                ElapsedTime timer = new ElapsedTime();
                actuation.wobbleArmDown();
                try {
                    Thread.sleep(600);
                    actuation.wobbleClawOpen();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            } else {
                actuation.wobbleClawClose();
                try {
                    Thread.sleep(300);
                    actuation.wobbleArmUp();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }*/

        if (actuation.hasRings()) {
            actuation.preheatShooter();
            telemetry.addLine("Rings present");
        } else actuation.killFlywheel();

       /* if(update1.circle())
            actuation.shoot(drive);
        if (update1.triangle())
            actuation.powerShots(drive);*/

        /* Uncomment for localization debugging
        telemetry.addData("x", drive.getPoseEstimate().getX());
        telemetry.addData("y", drive.getPoseEstimate().getY());
        telemetry.addData("heading", drive.getPoseEstimate().getHeading());*/

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

    static double powerScale(double power){
        return powerScale(power, 1);
    }

    static double powerScale(double power, double scale){
        if (power<=1) {
            if (power < 0)
                return -(scale * power * power);
            else
                return scale * power * power;
        }
        return 1;
    }
}
