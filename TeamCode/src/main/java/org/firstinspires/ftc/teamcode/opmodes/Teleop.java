package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class Teleop extends CommandOpMode {
    private Arm arm;
    private GamepadEx driver;
    @Override
    public void initialize() {
        this.arm = new Arm(this.hardwareMap.get(Servo.class, "arm"));
        this.driver = new GamepadEx(this.gamepad1);

        this.driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(arm.goToPos(Arm.ArmState.SCORE))
            .whenReleased(arm.goToPos(Arm.ArmState.HOME));

        register(arm);
    }


}
