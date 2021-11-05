package org.firstinspires.ftc.teamcode.robotAttachments;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OdometryPodServos {

    private static final double rightServoRaisePosition = 0; //TODO: Determine the actual value
    private static final double rightServoLowerPosition = 0; //TODO: Determine the actual value

    private static final double leftServoRaisePosition = 0;  //TODO: Determine the actual value
    private static final double leftServoLowerPosition = 0; //TODO: Determine the actual value

    private static final double horizontalServoRaisePosition = 0; //TODO: Determine the actual value
    private static final double horizontalServoLowerPosition = 0; //TODO: Determine the actual value

    Servo horizontalServo;
    Servo leftServo;
    Servo rightServo;

    OdometryPodServos(HardwareMap hardwareMap, String rightServoName, String leftServoName, String horizontalServoName){
        this.horizontalServo = hardwareMap.servo.get(horizontalServoName);
        this.rightServo = hardwareMap.servo.get(rightServoName);
        this.leftServo = hardwareMap.servo.get(leftServoName);
    }

    public void lower(){
        horizontalServo.setPosition(horizontalServoLowerPosition);
        rightServo.setPosition(rightServoLowerPosition);
        leftServo.setPosition(leftServoLowerPosition);
    }

    public void raise(){
        horizontalServo.setPosition(horizontalServoRaisePosition);
        rightServo.setPosition(rightServoRaisePosition);
        leftServo.setPosition(leftServoRaisePosition);
    }
}
