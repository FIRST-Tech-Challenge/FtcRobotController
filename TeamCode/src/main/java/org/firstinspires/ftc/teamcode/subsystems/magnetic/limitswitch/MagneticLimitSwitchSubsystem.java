package org.firstinspires.ftc.teamcode.subsystems.magnetic.limitswitch;

import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class MagneticLimitSwitchSubsystem extends SubsystemBase {

    //Arm Magnetic Limit Switch in Digital Port 2, Rev Hub 2
    private TouchSensor magneticLimitSwitch;

    private final Telemetry telemetry;


    public MagneticLimitSwitchSubsystem(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        magneticLimitSwitch = hwMap.get(TouchSensor.class, deviceName);
        this.telemetry = telemetry;
    }


    // Return the state of the Limit Switch, true/false
    public boolean armLimitSwitchPressed(){
        //telemetry.addData("ArmLimitSwitchPressed", magneticLimitSwitch.isPressed() );
        telemetry.update();

        return magneticLimitSwitch.isPressed();
    }
}
