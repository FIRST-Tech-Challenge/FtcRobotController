package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Drone {
    private final Servo droneServo1;
    private final Servo droneServo2;


    //value needs to be tested
    public static double shoot=0.8;

    public Drone(OpMode opMode) {
        droneServo1 = opMode.hardwareMap.servo.get("fourbarOne");
        droneServo1.setDirection(Servo.Direction.FORWARD);
        droneServo2 = opMode.hardwareMap.servo.get("fourbarTwo");
        droneServo2.setDirection(Servo.Direction.REVERSE);
    }

    public void Shoot(){
        droneServo1.setPosition(shoot);
        droneServo2.setPosition(shoot);
    }

    public void reset(){
        droneServo1.setDirection(Servo.Direction.REVERSE);
        droneServo2.setDirection(Servo.Direction.FORWARD);
        droneServo1.setPosition(shoot);
        droneServo2.setPosition(shoot);
    }



}
