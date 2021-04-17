package org.firstinspires.ftc.teamcode.tests.actuation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

import java.util.Vector;

@TeleOp
public class IntakeRingDetectionTest extends OpMode {

    DcMotor intake, backIntakeBelt;
    ColorSensor colorSensor;
    GamepadEventPS update;
    Actuation actuation;
    StandardMechanumDrive drive;
    int rings = 0;
    boolean inside = false;
    double runtime = 0;

    @Override
    public void init() {
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        intake = hardwareMap.dcMotor.get("intake");
        backIntakeBelt = hardwareMap.dcMotor.get("backIntakeBelt");
        update = new GamepadEventPS(gamepad1);
        drive = new StandardMechanumDrive(hardwareMap);
        actuation = new Actuation(hardwareMap,drive, null, this);
    }

    @Override
    public void loop() {

        if(gamepad1.left_trigger > 0.5)
            actuation.suck();

        if(gamepad1.right_trigger > 0.5)
            actuation.spitOut();

        if(gamepad1.right_trigger < 0.5 && gamepad1.left_trigger < 0.5)
            actuation.stopIntake();


        Vector2d input = new Vector2d(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x).rotated(drive.getPoseEstimate().getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

        /*if(colorSensor.red() > 50 && !inside) {
            rings += 1;
            inside = true;
        }
        else inside = false;*/

        if(colorSensor.red() > 50 && (getRuntime() - runtime) > .5) {
            rings += 1;
            runtime = getRuntime();
        }

        telemetry.addData("Intake Color R: ", colorSensor.red());
        telemetry.addData("Intake Color G: ", colorSensor.green());
        telemetry.addData("Intake Color B: ", colorSensor.blue());
        telemetry.addData("Ring present?", colorSensor.red() > 50 ? "yes" : "no");
        telemetry.addData("Rings intaked", rings);
        telemetry.update();

    }
}
