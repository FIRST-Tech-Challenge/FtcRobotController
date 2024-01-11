package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class plane implements Subsystem {
    private HardwareMap map;
    Telemetry telemetry;
    Servo servoPlane;

    public plane(HardwareMap map, Telemetry telemetry){
        this.map=map;
        this.telemetry=telemetry;
        servoPlane =map.servo.get("servoPlane");
    }
    @Override
    public void periodic() {
    }

    public Command shootPlane(){
        return new RunCommand(()->{
            servoPlane.setPosition(0);
            telemetry.addLine("aaa open");
        },this);
    };

//    todo: maybe add toggle option

}