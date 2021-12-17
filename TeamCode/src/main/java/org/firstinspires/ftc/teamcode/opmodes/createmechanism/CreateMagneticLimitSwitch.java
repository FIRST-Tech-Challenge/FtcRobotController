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

@Deprecated
public class CreateMagneticLimitSwitch {

    private MagneticLimitSwitchSubsystem magneticLimitSwitch;
    private final ArmSubsystem armSubsystem;
    private final HardwareMap hwMap;
    private final String deviceName;
    private final Telemetry telemetry;
    private Trigger mlsTrigger;


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
        ResetArmCount resetArmCount = new ResetArmCount(armSubsystem, telemetry);
        mlsTrigger = new Trigger(magneticLimitSwitch::armLimitSwitchPressed);

        mlsTrigger.whenActive(resetArmCount);

        telemetry.addData("Arm","set 0 limit switch");
        telemetry.update();

    }

    public Trigger getMagneticLimitSwitchTrigger(){
        return mlsTrigger;
    }
}
