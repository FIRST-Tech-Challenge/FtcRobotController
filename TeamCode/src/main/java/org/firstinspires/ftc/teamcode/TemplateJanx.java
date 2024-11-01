package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//ftclib library
public class TemplateJanx {
    public DcMotorEx fr = null;
    public DcMotorEx br = null;
    public DcMotorEx fl = null;
    public DcMotorEx bl = null;

    public Servo lc = null;
    public Servo rc = null;
    public DcMotorEx nod = null;

    public DcMotorEx ext = null;
    public DcMotorEx turn = null;

    public DcMotorEx sl = null;
    public DcMotorEx sr = null;

    public DcMotorEx motor = null;
    public Servo ser = null;

    HardwareMap hwMap = null;


    //clockwise from front right
    public TemplateJanx(HardwareMap h) {
        hwMap = h;
    }

    public void wheelInit(String frontRight, String backRight, String backleft, String frontLeft) {
        //when you make this in teleop or autonomous, you can import hardware.DcMotor with the keyword hardwareMap
        fr = hwMap.get(DcMotorEx.class, frontRight);
        br = hwMap.get(DcMotorEx.class, backRight);
        fl = hwMap.get(DcMotorEx.class, frontLeft);
        bl = hwMap.get(DcMotorEx.class, backleft);

        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //should it be brake?
        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
    }

    public void clawInit(String leftClaw, String rightClaw, String nodder) {
        nod = hwMap.get(DcMotorEx.class, nodder);
        nod.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nod.setDirection(DcMotor.Direction.FORWARD);
        lc = hwMap.get(Servo.class, leftClaw);
        rc = hwMap.get(Servo.class, rightClaw);
    }

    public void armInit(String armextend, String armTurn, String left, String right) {
        sl = hwMap.get(DcMotorEx.class, left);
        sr = hwMap.get(DcMotorEx.class, right);
        turn = hwMap.get(DcMotorEx.class, armTurn);

        turn.setDirection(DcMotor.Direction.FORWARD);
        turn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turn.setPower(0);

        sl.setDirection(DcMotor.Direction.FORWARD);
        sl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sl.setPower(0);

        sr.setDirection(DcMotor.Direction.REVERSE);
        sr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sr.setPower(0);
    }
    public void basicArmInit(String m, String s)
    {
        ser = hwMap.get(Servo.class, s);
        motor  = hwMap.get(DcMotorEx.class, m);

        if (ser == null){
            throw new NullPointerException("The servo must be populated in the robot configuration");
        }
        if(motor == null) {
            throw new NullPointerException("The motor must be populated in the robot configuration");
        }

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

}
