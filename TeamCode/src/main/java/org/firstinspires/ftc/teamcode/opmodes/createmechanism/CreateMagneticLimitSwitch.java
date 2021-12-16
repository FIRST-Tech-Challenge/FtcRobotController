package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.arm.NudgeArm;
import org.firstinspires.ftc.teamcode.commands.arm.ResetArmCount;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmLevel;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.magnetic.limitswitch.MagneticLimitSwitchSubsystem;

import java.util.HashMap;
import java.util.Map;

public class CreateMagneticLimitSwitch {

    private MagneticLimitSwitchSubsystem magneticLimitSwitch;
    private final ArmSubsystem armSubsystem;
    private final HardwareMap hwMap;
    private final String deviceName;
    private final Telemetry telemetry;


    private static final int NUDGE = 5;

    public CreateMagneticLimitSwitch(final HardwareMap hwMap, final String deviceName, final ArmSubsystem armSubsystem, Telemetry telemetry){
        this.deviceName = deviceName;
        this.armSubsystem = armSubsystem;
        this.hwMap = hwMap;
        this.telemetry = telemetry;

    }

    public CreateMagneticLimitSwitch(final HardwareMap hwMap, final String deviceName, final ArmSubsystem armSubsystem, Telemetry telemetry, boolean autoCreate){
        this.deviceName = deviceName;
        this.armSubsystem = armSubsystem;
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        if (autoCreate) create();

    }

    public void create(){

        magneticLimitSwitch = new MagneticLimitSwitchSubsystem(hwMap, deviceName, telemetry);
        ResetArmCount resetArmCount = new ResetArmCount(armSubsystem, () ->magneticLimitSwitch.armLimitSwitchPressed(), telemetry);
        resetArmCount.schedule();
    }

    public MagneticLimitSwitchSubsystem getMagneticLimitSwitch(){
        return magneticLimitSwitch;
    }
}
