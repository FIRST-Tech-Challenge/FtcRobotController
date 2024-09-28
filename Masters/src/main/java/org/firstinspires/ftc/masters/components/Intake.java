//package org.firstinspires.ftc.masters.components;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class Intake implements Component {
//
//    CRServo leftIntake;
//    CRServo rightIntake;
//    HardwareMap hardwareMap;
//
//    public Intake(HardwareMap hardwareMap){
//        this.hardwareMap=hardwareMap;
//        initializeHardware();
//    }
//
//    public void initializeHardware() {
//        leftIntake = hardwareMap.crservo.get("1");
//        rightIntake = hardwareMap.crservo.get("0");
//    }
//
//    public void reverse() {
//        leftIntake.setPower(-1);
//        rightIntake.setPower(1);
//    }
//
//    public void forward() {
//        leftIntake.setPower(1);
//        rightIntake.setPower(-1);
//    }
//
//    public void stop() {
//        leftIntake.setPower(0);
//        rightIntake.setPower(0);
//    }
//
//}
