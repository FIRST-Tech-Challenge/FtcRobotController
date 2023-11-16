package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class gripper implements Subsystem {
    private HardwareMap map;
    Servo serv0;
    Servo serv1;
    public gripper(HardwareMap map){
        this.map=map;
        serv0=map.servo.get("serv0");
        serv1=map.servo.get("serv1");
    }
    @Override
    public void periodic() {
    }
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }
    public Command openGripper(){
        return new InstantCommand(()->{
            serv0.setPosition(0);
            serv1.setPosition(1);
        });
    };
    public Command closeGripper(){
        return new InstantCommand(()->{
            serv0.setPosition(1);
            serv1.setPosition(0);
        });
    };
//    todo: maybe add toggle option

}