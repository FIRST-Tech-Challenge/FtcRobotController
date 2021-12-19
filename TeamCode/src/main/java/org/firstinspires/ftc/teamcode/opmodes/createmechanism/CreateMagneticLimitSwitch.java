package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.commands.arm.ResetArmCount;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.magnetic.limitswitch.MagneticLimitSwitchSubsystem;

public class CreateMagneticLimitSwitch {

    private MagneticLimitSwitchSubsystem magneticLimitSwitch;
    private final HardwareMap hwMap;
    private final String deviceName;
    private final Telemetry telemetry;
    private Trigger mlsTrigger;


    public CreateMagneticLimitSwitch(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.telemetry = telemetry;

    }

    public CreateMagneticLimitSwitch(final HardwareMap hwMap, final String deviceName, Telemetry telemetry, boolean autoCreate){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        if (autoCreate) create();

    }

    public void create(){

        magneticLimitSwitch = new MagneticLimitSwitchSubsystem(hwMap, deviceName, telemetry);

    }

    public MagneticLimitSwitchSubsystem getMagneticLimitSwitchTrigger(){
        return magneticLimitSwitch;
    }
}
