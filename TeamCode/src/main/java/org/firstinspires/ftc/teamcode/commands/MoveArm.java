package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;

import org.firstinspires.ftc.robotcore.external.Telemetry;




public class MoveArm extends CommandBase {
    private final ArmSub armSub;
    private final Telemetry telemetry;
    private final double speed;

    public MoveArm(ArmSub armSub, Telemetry telemetry, double speed){
        this.armSub = armSub;
        this.speed = speed;
        this.telemetry = telemetry;
        addRequirements(this.armSub);
    }

    @Override
    public void execute() {
        armSub.move(speed);
    }
}
