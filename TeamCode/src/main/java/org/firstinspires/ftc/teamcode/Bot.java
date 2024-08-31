package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bot extends RobotDrive{
    double currspdmul = 0;
    double intkpwr = 0.0;
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
        rl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

        double targetServo = 0.73;
        int targetPos = 1;//Top height for rig
        if (servoHL.getPosition() < 0.73){
           rr.setTargetPosition(targetPos);//up
           rl.setTargetPosition(targetPos);
           rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           servoHL.setPosition(targetServo);//servo pos
           servoHR.setPosition(targetServo);
           rr.setTargetPosition(0);//down
           rl.setTargetPosition(0);
           rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        else if(servoHL.getPosition() >= 0.73){
            rr.setTargetPosition(targetPos);//up
            rl.setTargetPosition(targetPos);
            rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoHL.setPosition(0);//servo pos
            servoHR.setPosition(0);
            rr.setTargetPosition(0);//down
            rl.setTargetPosition(0);
            rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    //intakeonoff


    //lefttrigger for power changing
    public double setspeed(double LTstats, double ogpwr){
        if (LTstats > 0.5){
            currspdmul = 1;
            return ogpwr;
        }
        else if (LTstats < 0.5 && LTstats > 0.0) {
            currspdmul =  0.6;
            return ogpwr * currspdmul;
        }
        else{
            currspdmul = 0.8;
            return ogpwr * currspdmul;
        }
    }


}
