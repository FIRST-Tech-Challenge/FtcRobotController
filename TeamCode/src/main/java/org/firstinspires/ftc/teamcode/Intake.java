package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private CRServo Intakes = null;
    private Servo graber1 = null;
    private Servo graber2 = null;
    private LinearOpMode opmode=null;

    public  Intake (){
    }
    public void init (LinearOpMode opMode){
        HardwareMap hwMap;
        opmode = opMode;
        hwMap=opMode.hardwareMap;

        Intakes=hwMap.crservo.get("Intakes");
        graber1=hwMap.servo.get("graber1");
        graber2=hwMap.servo.get("graber2");

        Intakes.setPower(0);

    }
    public void intake() {
        Intakes.setPower(1);
    }
    public void eject(){
        Intakes.setPower(-1);
    }
    public void transport(double speed){
        Intakes.setPower(0);
    }
}
