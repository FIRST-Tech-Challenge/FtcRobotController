package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Drone {
    private final Servo droneServo;


    //value needs to be tested
    public static double shoot=0.8;

    public Drone(OpMode opMode) {
        droneServo = opMode.hardwareMap.servo.get("droneServo");
        droneServo.setDirection(Servo.Direction.FORWARD);
    }

    public void shoot(){
        droneServo.setPosition(shoot);
    }

    public void reset(){
        droneServo.setDirection(Servo.Direction.REVERSE);
        droneServo.setPosition(shoot);
    }

}
