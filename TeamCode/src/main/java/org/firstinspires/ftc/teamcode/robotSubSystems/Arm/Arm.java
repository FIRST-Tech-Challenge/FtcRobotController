package org.firstinspires.ftc.teamcode.robotSubSystems.Arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    public static Servo armServo;
    static double targetPosition = 0;

    public static void init(HardwareMap hardwareMap) {
        //rightWristServo = hardwareMap.servo.get("rightWristServo");
        armServo = hardwareMap.servo.get("armServo");

    }
    public static void operate() {
        armServo.setPosition(targetPosition);
    }
    public static  void  open (Telemetry telemtry){
        targetPosition = 0.6;
//        armServo.setPosition(armServo.getPosition()+0.025);
    }
    public static void close (Telemetry telemtry){
        targetPosition = 0;
//        armServo.setPosition(armServo.getPosition()+0.025);
    }

    //    public static void operate(WristState state) {
//        switch (state) {
//            case TRANSFER:
//                leftWristServo.setPosition(0.01);
//                //rightWristServo.setPosition(0);
//                break;
//            case INTAKE:
//                leftWristServo.setPosition(0.735);
//                //rightWristServo.setPosition(0.6);
//                break;
//            case DEPLETE:
//                leftWristServo.setPosition(0.67);
//                //rightWristServo.setPosition(0.5);

}