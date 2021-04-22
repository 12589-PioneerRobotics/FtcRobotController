package org.firstinspires.ftc.teamcode.tests.actuation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Deadwheel Test", group = "Tests")
public class DeadWheelTest extends OpMode {
    DcMotor rightDeadwheel;
    DcMotor leftDeadwheel;
    DcMotor sidewaysDeadwheel;

    @Override
    public void init() {
        leftDeadwheel = hardwareMap.dcMotor.get("backIntakeBelt");
        rightDeadwheel = hardwareMap.dcMotor.get("backLeft");
        sidewaysDeadwheel = hardwareMap.dcMotor.get("sideways_deadwheel");

        rightDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sidewaysDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDeadwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDeadwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sidewaysDeadwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("left (left)", leftDeadwheel.getCurrentPosition());
        telemetry.addData("right (right)", rightDeadwheel.getCurrentPosition());
        telemetry.addData("sideways (sideways)", sidewaysDeadwheel.getCurrentPosition());
        telemetry.update();
    }
}
