package org.firstinspires.ftc.teamcode.core.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.jetbrains.annotations.NotNull;

/**
 * Simple wrapper class for easy togglable controls of a PlayStation controller, using Toggle.OneShot.
 * Every button/bumper on the PS controller is mapped to a separate Toggle.OneShot instance.
 */
public class GamepadEventPS extends Toggle.OneShot {
    Toggle.OneShot circle, square, cross, triangle, dPadLeft, dPadRight, dPadUp, dPadDown, leftBumper, rightBumper, leftStickButton, rightStickButton, share, ps;

    Gamepad gamepad;

    public GamepadEventPS(@NotNull Gamepad gamepad) {
        circle = new Toggle.OneShot();
        square = new Toggle.OneShot();
        cross = new Toggle.OneShot();
        triangle = new Toggle.OneShot();
        dPadRight = new Toggle.OneShot();
        dPadLeft = new Toggle.OneShot();
        dPadUp = new Toggle.OneShot();
        dPadDown = new Toggle.OneShot();
        leftBumper = new Toggle.OneShot();
        rightBumper = new Toggle.OneShot();
        leftStickButton = new Toggle.OneShot();
        rightStickButton = new Toggle.OneShot();
        share = new Toggle.OneShot();
        ps = new Toggle.OneShot();

        this.gamepad = gamepad;
    }

    public boolean cross() { return cross.update(gamepad.cross); }
    public boolean circle() { return circle.update(gamepad.circle); }
    public boolean square() { return square.update(gamepad.square); }
    public boolean triangle() { return triangle.update(gamepad.triangle); }
    public boolean dPadDown() { return dPadDown.update(gamepad.dpad_down); }
    public boolean dPadUp() { return dPadUp.update(gamepad.dpad_up); }
    public boolean dPadRight() { return dPadRight.update(gamepad.dpad_right); }
    public boolean dPadLeft() { return dPadLeft.update(gamepad.dpad_left); }
    public boolean leftBumper() {return leftBumper.update(gamepad.left_bumper); }
    public boolean rightBumper() {return rightBumper.update(gamepad.right_bumper);}
    public boolean leftStickButton() {return leftStickButton.update(gamepad.left_stick_button);}
    public boolean rightStickButton() {return rightStickButton.update(gamepad.right_stick_button);}
    public boolean share() {return share.update(gamepad.share);}
    public boolean ps() {return share.update(gamepad.ps);}


}
