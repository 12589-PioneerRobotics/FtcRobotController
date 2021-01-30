package org.firstinspires.ftc.teamcode.core.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.jetbrains.annotations.NotNull;

public class GamepadEventPS extends Toggle.OneShot {
    Toggle.OneShot circle, square, x, triangle, dPadLeft, dPadRight, dPadUp, dPadDown, leftBumper, rightBumper;

    Gamepad gamepad;

    public GamepadEventPS(@NotNull Gamepad gamepad) {
        circle = new Toggle.OneShot();
        square = new Toggle.OneShot();
        x = new Toggle.OneShot();
        triangle = new Toggle.OneShot();
        dPadRight = new Toggle.OneShot();
        dPadLeft = new Toggle.OneShot();
        dPadUp = new Toggle.OneShot();
        dPadDown = new Toggle.OneShot();
        leftBumper = new Toggle.OneShot();
        rightBumper = new Toggle.OneShot();

        this.gamepad = gamepad;

    }

    public boolean x() { return x.update(gamepad.x); }
    public boolean circle() { return circle.update(gamepad.circle); }
    public boolean square() { return square.update(gamepad.square); }
    public boolean triangle() { return triangle.update(gamepad.triangle); }
    public boolean dPadDown() { return dPadDown.update(gamepad.dpad_down); }
    public boolean dPadUp() { return dPadUp.update(gamepad.dpad_up); }
    public boolean dPadRight() { return dPadRight.update(gamepad.dpad_right); }
    public boolean dPadLeft() { return dPadLeft.update(gamepad.dpad_left); }
    public boolean leftBumper() {return leftBumper.update(gamepad.left_bumper); }
    public boolean rightBumper() {return rightBumper.update(gamepad.right_bumper);}

}
