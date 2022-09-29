package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.event loop.OpMode;
import com.qualcomm.robotcore.event.loop.opmode.Teleop;

@Teleop()
Public classs GamepadOpMode extends OpMode {
    @override
    public void init() {
    }

    @override
    public void loop () {
        telemetry.addData("Left stick x", gamepad.left_stick_x);
        telemetry.addData("Left stick y", gamepad.left_stick_y);
        telemetry.addData("Anjali", "Mathews");
        telemetry.addData("A button", gamepad1.a);
    }
}