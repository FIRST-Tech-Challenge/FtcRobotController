package org.firstinspires.ftc.teamcode.powerplay;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotbase.RobotEx;

public class PowerPlayRobot extends RobotEx {
    private ClawSubsystem claw;
    private SliderSubsystem slider;

    public PowerPlayRobot(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driverOp,
                          GamepadEx toolOp) {
        super(hardwareMap, telemetry, driverOp, toolOp);
    }

    @Override
    public void initMechanisms(HardwareMap hardwareMap) {
        ////////////////////////////////////////// Claw //////////////////////////////////////////
        claw = new ClawSubsystem(hardwareMap);
        toolOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ClawCommand(claw));

        ////////////////////////////////////////// Slider //////////////////////////////////////////
        slider = new SliderSubsystem(hardwareMap);

        //Levels
        toolOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.ONE));
        toolOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.TWO));
        toolOp.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new SliderCommand(slider, SliderSubsystem.Level.THREE));

        //Manual Height Adjustment
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(new SliderManualCommand(slider, -1));
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(new SliderManualCommand(slider, 1));
    }
}