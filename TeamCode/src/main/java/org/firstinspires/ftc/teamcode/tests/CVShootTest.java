package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.ActuationConstants;
import org.firstinspires.ftc.teamcode.core.CVShooting;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_REST;
import static org.firstinspires.ftc.teamcode.core.ActuationConstants.FEEDER_YEET;

public class CVShootTest extends OpMode {
    CVShooting frame;
    GamepadEventPS update;
    StandardMechanumDrive drive;
    Actuation actuation;
    int currentTargetIndex = 0;

    @Override
    public void init() {
        frame = new CVShooting(ActuationConstants.Target.values()[currentTargetIndex]);
        update = new GamepadEventPS(gamepad1);
        drive = new StandardMechanumDrive(hardwareMap);
        actuation = new Actuation(null, drive);
    }

    @Override
    public void loop() {

        if (update.dPadLeft()) {
            if (currentTargetIndex != 0)
                currentTargetIndex -= 1;
        }
        if (update.dPadRight()) {
            if (currentTargetIndex != ActuationConstants.Target.values().length - 1)
                currentTargetIndex += 1;
        }

        if (update.cross()) {

            while (!frame.isAligned()) {
                drive.setWeightedDrivePower(new Pose2d(0, 0, 0.2));
            }
            // Shoot
            actuation.preheatShooter(-4.0);
            try {
                Thread.sleep(500);
                actuation.feeder.setPosition(FEEDER_REST);
                Thread.sleep(500);
                actuation.feeder.setPosition(FEEDER_YEET);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        frame.setTarget(ActuationConstants.Target.values()[currentTargetIndex]);

        telemetry.addLine("Press x to turn");
        telemetry.addData("Current target", frame.getTarget().toString());
        telemetry.addLine(frame.isAligned() ? "Aligned" : "Not aligned");
        telemetry.update();
    }
}
