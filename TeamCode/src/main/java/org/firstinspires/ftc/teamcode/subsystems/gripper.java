package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class gripper implements Subsystem {
    private HardwareMap map;
    Telemetry telemetry;
    Servo serv0;
    Servo serv1;
    public gripper(HardwareMap map, Telemetry telemetry){
        this.map=map;
        this.telemetry=telemetry;
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
        return new RunCommand(()->{
            serv0.setPosition(0);
            serv1.setPosition(180);
            telemetry.addLine("aaa open");
        },this);
    };
    public Command closeGripper(){
        return new RunCommand(()->{
            serv0.setPosition(180);
            serv1.setPosition(0);
            telemetry.addLine("aaa close");
        },this);
    };
//    todo: maybe add toggle option

}