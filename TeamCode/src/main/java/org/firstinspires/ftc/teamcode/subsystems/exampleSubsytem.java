package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class exampleSubsytem implements Subsystem {
    MotorEx dc;
    private HardwareMap map;
    public exampleSubsytem(HardwareMap map){
        this.map=map;
        dc= new MotorEx(map,"this name1");
    }

    @Override
    public void periodic() {
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }



}
