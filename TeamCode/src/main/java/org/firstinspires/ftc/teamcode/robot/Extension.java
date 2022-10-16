package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


public class Extension {
    private Telemetry telemetry;
    public Servo extensionServo;

    static final double MM_TO_INCHES = 0.0393700787;

    static final double MOVE_OUT_SPEED = 0.5;
    static final double MOVE_BACK_SPEED = 0.5;

    static final double MINIMUM_CLEARANCE_DISTANCE = 95.875 * MM_TO_INCHES;

    public final double EXTENSION_POSITION_RESET = 0;
    public final int MAXIMUM_EXTEND_VALUE = 2520;

    public Extension(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        extensionServo = (Servo) hwMap.servo.get("Extension");
    }

    //speed is the value coming from controller
    public void Extend(double speed){
        double currentPosition;
        currentPosition = extensionServo.getPosition();

        //moving in increments of 250
        double targetPosition = currentPosition + speed*250;
        if (targetPosition > MAXIMUM_EXTEND_VALUE)
            targetPosition = MAXIMUM_EXTEND_VALUE;

        extensionServo.setPosition(targetPosition/MAXIMUM_EXTEND_VALUE);
    }

    //speed is the value coming from controller
    public void Retract (double speed) {
        double currentPosition;
        currentPosition = extensionServo.getPosition();

        //moving in increments of 250
        double targetPosition = currentPosition - speed*250;
        if (targetPosition < 0)
            targetPosition = 0;
        
        extensionServo.setPosition(targetPosition/MAXIMUM_EXTEND_VALUE);
    }
}
