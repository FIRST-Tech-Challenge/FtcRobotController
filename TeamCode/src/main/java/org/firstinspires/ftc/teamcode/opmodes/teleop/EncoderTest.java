package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Drives a pushbot with teleop control.
 */
@TeleOp(name = "EncoderTest", group="TeleOp")
public class EncoderTest extends BaseOpMode {

    public void init() {
        super.init();
    }

    /**
     * Drives the motors based on the joystick sticks
     * Left trigger engages slow-drive
     */

    public void loop() {
        for (DcMotor motor : this.driveSystem.motors.values()) {
            telemetry.addData("ticks: ", motor.getCurrentPosition());
        }
        telemetry.update();
    }
}
