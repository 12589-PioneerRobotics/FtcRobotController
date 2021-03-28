package org.firstinspires.ftc.teamcode.core.gamepad;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Utility class to easily create toggle controls in TeleOp or any other controller-based OpMode.
 * Instead of having to hold a button to perform an action (such as holding the left bumper to move
 * a servo to a position), one can check the toggle condition by mapping a OneShot instance to one
 * control.
 * NOTE: GameEventsPS is a helper class that makes this process simple.
 */
public class Toggle {

    private OneShot oneShot;
    private boolean oldValue;

    public Toggle() {
        oneShot = new OneShot();
    }

    public boolean update(boolean newValue) {
        if (oneShot.update(newValue)) {
            oldValue = !oldValue;
        }
        return oldValue;
    }

    public static class OneShot {
        private boolean oldValue;

        public boolean update(boolean newValue) {
            if (newValue) {
                if (!oldValue) {
                    oldValue = true;
                    return true;
                }
            } else {
                oldValue = false;
            }
            return false;
        }
    }

    public static class TimeToggle {
        private final Toggle toggle = new Toggle();
        OneShot oneShot = new OneShot();
        private final OpMode opMode;
        private final double targetDeltaTime;
        private boolean wasPressed;
        private double startTime;

        public TimeToggle(OpMode opMode, double time) {
            this.opMode = opMode;
            targetDeltaTime = time;
        }

        public boolean update(boolean newValue) {
            if (oneShot.update(newValue)) {
                startTime = opMode.getRuntime();
            }
            return toggle.update(newValue && opMode.getRuntime() - startTime > targetDeltaTime);
        }
    }
}