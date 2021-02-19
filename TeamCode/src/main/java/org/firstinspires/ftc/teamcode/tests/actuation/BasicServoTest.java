package org.firstinspires.ftc.teamcode.tests.actuation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.gamepad.GamepadEventPS;

@TeleOp
public class BasicServoTest extends OpMode {
    Servo servo;
    Servo servo2;


    GamepadEventPS update;
    double position = .5;
    double increment = .025;

    double position2 = 0;
    double increment2 = .05;
    /*ValueCycler positionIncrements = new ValueCycler(gamepad1,
            new double[]{.01, .05, .1, .2, .3},
            ValueCycler.CONTROL_PAIRS.D_PAD_VERTICAL, update);*/
    int index = 0;

    @Override
    public void init() {

        servo = hardwareMap.servo.get("wobbleGrab");
        servo2 = hardwareMap.servo.get("wobbleArm");
//        servo2.getController().pwmDisable();

        update = new GamepadEventPS(gamepad1);
        
    }

    @Override
    public void loop() {

//        increment = positionIncrements.update();

        if (update.circle())
            servo.setPosition(0);
        if (update.cross())
            servo.setPosition(1);

        if (update.dPadLeft())
            position -= increment;
        if (update.dPadRight())
            position += increment;

        if(update.dPadDown())
            position2 -= increment;
        if(update.dPadUp())
            position2 += increment;

        servo.setPosition(position);
        servo2.setPosition(position2);
        telemetry.addData("Increment level", increment);
//        telemetry.addData("Press a and b to move to either ends", "");
        telemetry.addData("Current position wobble grab", servo.getPosition());
        telemetry.addData("Current position wobble arm", servo2.getPosition());
        telemetry.update();
    }
}
