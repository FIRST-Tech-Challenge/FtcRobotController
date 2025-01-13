package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

//import com.qualcomm.robotcore.hardware.Servo;


//port=0
//linear rail= porrt 2
//open and closing claw=1 Port


public class ArmRotating extends LinearOpMode {
    private String armservo="";
    @Override
    public void runOpMode() {

         Servo armRotationServo = hardwareMap.get(Servo.class, armservo);

    }




}
