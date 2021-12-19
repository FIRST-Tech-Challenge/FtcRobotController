package org.firstinspires.ftc.teamcode.opmodes.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.arm.ResetArmCount;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.magnetic.limitswitch.MagneticLimitSwitchSubsystem;

public class CreateMagneticLimitSwitchTrigger {

    private  MagneticLimitSwitchSubsystem magneticLimitSwitch;
    private final ArmSubsystem armSubsystem;

    private final HardwareMap hwMap;
    private final String deviceName;
    private final Telemetry telemetry;
    private Trigger mlsTrigger;


    public CreateMagneticLimitSwitchTrigger(final HardwareMap hwMap, final String deviceName, final ArmSubsystem arm,  Telemetry telemetry){
        this.deviceName = deviceName;
        armSubsystem = arm;
        this.hwMap = hwMap;
        this.telemetry = telemetry;

    }

    public CreateMagneticLimitSwitchTrigger(final HardwareMap hwMap, final String deviceName, final ArmSubsystem arm, Telemetry telemetry, boolean autoCreate){
        this.deviceName = deviceName;
        armSubsystem = arm;
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        if (autoCreate) create();

    }

    public void create(){

        magneticLimitSwitch = new MagneticLimitSwitchSubsystem(hwMap, deviceName, telemetry);
        mlsTrigger = new Trigger(() -> magneticLimitSwitch.armLimitSwitchPressed() && armSubsystem.getCanResetArmFlag());

        telemetry.addData("trigger created","trigger created");
        telemetry.update();

    }

    public Trigger getMagneticLimitSwitchTrigger(){
        return mlsTrigger;
    }
}
