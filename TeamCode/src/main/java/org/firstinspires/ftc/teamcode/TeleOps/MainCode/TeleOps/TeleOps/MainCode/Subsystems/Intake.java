package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    private DcMotorEx intakeMotor;
    private GamepadEx gamepad;

    public Intake(GamepadEx gamepad, HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        this.gamepad = gamepad;
    }

    public void intake() {
        double power = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        intakeMotor.setPower(power);
    }

    public void outtake() {
        double power = -gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        intakeMotor.setPower(power);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
