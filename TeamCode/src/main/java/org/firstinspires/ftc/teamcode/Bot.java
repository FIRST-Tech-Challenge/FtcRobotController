package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bot extends RobotDrive{
    public DcMotor intake;
    public DcMotorEx rr;
    public DcMotorEx rl;
   public Servo servoTransfer;
    public Servo servoHL;
    public Servo servoHR;
    public Servo servoSLB;
    public Servo servoSLT;
    public Servo servoSRT;
    public Servo servoSRB;
    public Servo servoFlight;
    public Servo servoRR;
    public Servo servoRL;
    public void init(HardwareMap hardwareMap) {
//        fl = hardwareMap.dcMotor.get("fl"); //port 3 EH - RED (In RobotDrive)
//        bl = hardwareMap.dcMotor.get("bl"); //port 1 CH - YELLOW (In RobotDrive)
//        br = hardwareMap.dcMotor.get("br"); //port 0 CH - ORANGE (In RobotDrive)
//        fr = hardwareMap.dcMotor.get("fr"); //port 0 EH - BLUE (In RobotDrive)
        intake = (DcMotor) hardwareMap.dcMotor.get("intake"); //port 1 EH - PURPLE
        rr = (DcMotorEx) hardwareMap.dcMotor.get("rr"); // rigging right, port 3 CH - DARK GREEN
        rl = (DcMotorEx) hardwareMap.dcMotor.get("rl"); // rigging left, port 2 CH - LIGHT GREEN


        servoTransfer = hardwareMap.servo.get("transfer"); // port 3 EH, Transfer
        servoHL = hardwareMap.servo.get("hl"); // port 0 CH, Hook Left
        servoHR = hardwareMap.servo.get("hr"); // port 5 CH, Right Hook
        servoSLB = hardwareMap.servo.get("slb"); // port 2 CH, Slides Left Bottom
        servoSLT = hardwareMap.servo.get("slt"); // port 1 CH, Slides Left Top
        servoSRT = hardwareMap.servo.get("srt"); // port 4 CH, Slides Right Top
        servoSRB = hardwareMap.servo.get("srb"); // port 3 CH, Slides Right Bottom
        servoFlight = hardwareMap.servo.get("flight"); //port 1 EH, Flight
        servoRR = hardwareMap.servo.get("rigr"); // port 5 EH, Rigging Right
        servoRL = hardwareMap.servo.get("rigl"); //port 0 EH, Rigging Left

        //add servos and sensors to be used in all autonmous and teleop classes
        //add all the servo positions and stuff
        super.init(hardwareMap); //runs the robot drive part
    }

    public void TransferToggle() {
        if (servoTransfer.getPosition() < 0.11) {
            servoTransfer.setPosition(0.5);
        } else {
            servoTransfer.setPosition(0.1);
        }

    }

    //rigging
    public void rig() {
        // up: 0.7
        // down:
        double position = 0.0;
        if (position != 0.7) {
            position = 0.7;
        } else {
            position = 0.0;
        }

        servoHL.setPosition(position);
        servoHR.setPosition(position);
    }

    //intakeonoff
    public void intake(boolean rev){// rev specifies whether the intake should be reversed true = reversed false = normal
        double intkpwr = 0.0;// intake power
        if (rev == false){
           if (intkpwr != 1.0){
               intkpwr = 1.0;
           }
           else{
               intkpwr = 0.0;
           }
        }
        else{
            if (intkpwr != -1.0){
                intkpwr = -1.0;
            }
            else{
                intkpwr = 0.0;
            }
        }
    }

    //lefttrigger for power changing
    public double setspeed(double LTstats, double ogpwr){
        if (LTstats > 0.5){
            return ogpwr;
        }
        else if (LTstats < 0.5 && LTstats > 0.0) {
            return ogpwr * 0.6;
        }
        else{
            return ogpwr * 0.8;
        }
    }
}
