package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.BTCommand;

import java.util.function.DoubleSupplier;

public class Gripper implements Subsystem {
    private HardwareMap map;
    Telemetry telemetry;
    Servo serv0;
    Servo serv1;
//    ServoEx
    public Gripper(HardwareMap map, Telemetry telemetry){
        this.map=map;
        this.telemetry=telemetry;
        serv0=map.servo.get("gripper0");
        serv1=map.servo.get("gripper1");
        serv1.getController().pwmEnable();
        serv0.getController().pwmEnable();
        register();
    }
    @Override
    public void periodic() {
        FtcDashboard.getInstance().getTelemetry().addData("serv1 angle",serv1.getPosition());
        FtcDashboard.getInstance().getTelemetry().addData("serv0 angle",serv0.getPosition());
        FtcDashboard.getInstance().getTelemetry().update();
    }
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }
    public Command openGripper(){
        return new RunCommand(()->{
            serv0.setPosition(0.8);
            serv1.setPosition(1);
        },this);
    };
    public Command closeGripper(){
        return new RunCommand(()->{
            serv0.setPosition(0.35);
            serv1.setPosition(0);
        },this);
    };
    public Command enablePWM(){
        return  new InstantCommand(()->{
            serv1.getController().pwmEnable();
            serv0.getController().pwmEnable();
        });
    }
    public Command manual(){
        return  enablePWM().andThen(new org.firstinspires.ftc.teamcode.utils.RunCommand(()->{
            serv0.setPosition(Constants.Gripper.Gripperpwm.serv0);
            serv1.setPosition(Constants.Gripper.Gripperpwm.serv1);

        }));
    }
    public Command stop(){
        return new InstantCommand(()->{
            serv1.getController().pwmDisable();
            serv0.getController().pwmDisable();
        });
    }

//    todo: maybe add toggle option

}