package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TevelRobot {
    Elevator elevator;
    Wheels wheels;

    public Elevator getElevator() {
        return elevator;
    }

    public Wheels getWheels() {
        return wheels;
    }

    public TevelRobot(LinearOpMode opMode){
        elevator = new Elevator(opMode.hardwareMap, opMode.telemetry);
         wheels = new Wheels(opMode);
    }
    public void init() {
        elevator.initMotors();
    }

}
