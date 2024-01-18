package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.RunCommand;

public class Gripper extends SubsystemBase {
    private HardwareMap map;
    private Telemetry telemetry;
    private Servo serv0;
    private Servo serv1;
    private boolean isOpen1;
    private boolean isOpen2;


    public Gripper(HardwareMap map, Telemetry telemetry) {
        this.map = map;
        this.telemetry = telemetry;
        serv0 = map.servo.get("serv0");
        serv1 = map.servo.get("serv1");
        isOpen1 = false;
        isOpen2 = false;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        super.setDefaultCommand(defaultCommand);
    }

    public Command openGripper() {
        return openGripper1().alongWith(openGripper2());

    }

    public Command closeGripper() {
        return closeGripper1().alongWith(closeGripper2());
    }

    public Command toggleGripper(){
        return toggleGripper1().alongWith(toggleGripper2());
    }

    public Command toggleGripper1(){
        if (isOpen1) return closeGripper1();
        else return openGripper1();
    }

    public Command toggleGripper2(){
        if (isOpen2) return closeGripper2();
        else return openGripper2();
    }

    public Command openGripper1() {
        return new RunCommand(() -> {
            serv0.setPosition(0);
            isOpen1 = true;
            telemetry.addLine("aaa open");
        }, this);
    }

    public Command closeGripper1() {
        return new RunCommand(() -> {
            serv0.setPosition(180);
            isOpen1 = false;
            telemetry.addLine("aaa close");
        }, this);
    }

    public Command closeGripper2() {
        return new RunCommand(() -> {
            serv1.setPosition(0);
            isOpen2 = false;
            telemetry.addLine("aaa close");
        }, this);
    }

    public Command openGripper2() {
        return new RunCommand(() -> {
            serv1.setPosition(180);
            isOpen2 = true;
            telemetry.addLine("aaa open");
        }, this);
    }
}

