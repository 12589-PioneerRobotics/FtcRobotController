package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.POWER_SHOT_LEFT;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.Target.TOWER_GOAL;

@TeleOp(name = "Powershots Test")
public class PowershotsTest extends LinearOpMode {
    StandardMechanumDrive drive;
    Actuation actuation;
    GamepadEventPS update1;
    boolean targettingTowerGoal = true;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new StandardMechanumDrive(hardwareMap);
        actuation = new Actuation(this, drive);
        update1 = new GamepadEventPS(gamepad1);

        actuation.cvShootingInit();

        waitForStart();

        while(opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            // Intake functionality
            if (gamepad1.right_trigger > .5) actuation.suck();
            else if (gamepad1.left_trigger > .5) actuation.spitOut();
            else if (gamepad1.right_trigger < .5 && gamepad2.left_trigger < .5) actuation.stopIntake();

            if(update1.square()) {
                targettingTowerGoal = !targettingTowerGoal;
                if(targettingTowerGoal)
                    actuation.preheatShooter(TOWER_GOAL);
                else actuation.preheatShooter(POWER_SHOT_LEFT);
            }

            if(update1.circle()) {
                actuation.powerShots();
            }

        }

        telemetry.addData("Targeting", targettingTowerGoal ? "Tower" : "Power Shots");
        telemetry.update();

    }
}
