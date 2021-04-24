package org.firstinspires.ftc.teamcode.tests.actuation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Actuation;
import org.firstinspires.ftc.teamcode.core.StandardMechanumDrive;
import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

import java.util.Vector;

import static org.firstinspires.ftc.teamcode.TeleOp.powerScale;

@TeleOp
public class BasicIntakeTest extends OpMode {

    DcMotor intake;
    Servo frontIntake;
    ColorSensor colorSensor;
    GamepadEventPS update;
    Actuation actuation;
    StandardMechanumDrive drive;
    int rings = 0;
    boolean inside = false;
    double runtime = 0;
    double frontIntakePosition = 0.25;
    double speedMultiplier = .35;
    double increment = 0.01;

    @Override
    public void init() {
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);
        intake = hardwareMap.dcMotor.get("intake");
        update = new GamepadEventPS(gamepad1);
        drive = new StandardMechanumDrive(hardwareMap);
        actuation = new Actuation(hardwareMap,drive, null, this);
        frontIntake = hardwareMap.servo.get("intakeTension");
    }

    @Override
    public void loop() {

        if(gamepad1.left_trigger > 0.5)
            actuation.suck();

        if(gamepad1.right_trigger > 0.5)
            actuation.spitOut();

        if(gamepad1.right_trigger < 0.5 && gamepad1.left_trigger < 0.5)
            actuation.stopIntake();

        if(update.dPadDown())
            frontIntakePosition -= increment;
        if(update.dPadUp())
            frontIntakePosition += increment;

        frontIntake.setPosition(frontIntakePosition);

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x).rotated(-drive.getPoseEstimate().getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

        if(update.dPadRight())
            speedMultiplier += increment;
        if(update.dPadLeft())
            speedMultiplier -= increment;

        /*drive.setDrivePower(
                new Pose2d(
                        powerScale(gamepad1.left_stick_y, speedMultiplier),
                        powerScale(gamepad1.left_stick_x, speedMultiplier),
                        -powerScale(gamepad1.right_stick_x, speedMultiplier)
                )
        );*/


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
        telemetry.addData("Front Intake Servo Pos", frontIntake.getPosition());
        telemetry.addData("speed scale", speedMultiplier);
        telemetry.update();

    }
}
