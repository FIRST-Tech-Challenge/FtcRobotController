package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class Elevator {

    private LinearOpMode myOpMode = null;
    final double HOME_POWER = -0.1;
    double SLOW_LIFT = 0.2;
    double SLOW_LOWER = 0;
    double FAST_LIFT = 0.5;
    double FAST_LOWER = 0;
    int DEAD_BAND = 4;
    double WRIST_HOME_POSITION = 0.6;
    double HAND_HOME_POSITION = 0.8;
     double HAND_OPEN = 0.7;
     double HAND_CLOSE = 0.9;
     double ENCODER_TO_ANGLEM = 0.0864;
     double ENCODER_TO_ANGLEC = -30;
     double WRIST_TO_SERVOM = 0.00443;
     double WRIST_TO_SERVOC = 0.454;

    private DcMotorEx liftMaster;
    private DcMotorEx liftSlave;
    private List<DcMotorEx> motors;
    private Servo wrist; //smaller values tilt the wrist down
    private Servo hand;//smaller values open the hand more

    private int target = 0;
    private int     lastPosition = 0;
    private double  power    = 0;
    private boolean liftActive = false;

    public Elevator(LinearOpMode opMode) {

        myOpMode = opMode;
        liftMaster = myOpMode.hardwareMap.get(DcMotorEx.class, "lift_master");
        liftSlave = myOpMode.hardwareMap.get(DcMotorEx.class, "lefte");
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        hand = myOpMode.hardwareMap.get(Servo.class, "hand");
        motors = Arrays.asList(liftMaster, liftSlave);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMaster.setDirection(DcMotorSimple.Direction.FORWARD);
        liftSlave.setDirection(DcMotorSimple.Direction.REVERSE);
        setWristPosition(WRIST_HOME_POSITION);
        setHandPosition(HAND_HOME_POSITION);
    }

    public boolean runControlLoop() {
        double error = target - getPosition();
        if (error > DEAD_BAND*4) {
            setPower(FAST_LIFT);
        }
        else if (error > DEAD_BAND) {
            setPower(SLOW_LIFT);
        }
        else if (error < -DEAD_BAND*4) {
            setPower(FAST_LOWER);
        }
        else if (error < -DEAD_BAND) {
            setPower(SLOW_LOWER);
        }
        else {
            setPower(0);
        }
        //adjust the angle of the servo
        double armAngle = elevatorEncoderToAngle(getPosition());
        double servoAngle = -armAngle;
        double servoPosition = wristAngleToServo(servoAngle);
        setWristPosition(servoPosition);
        myOpMode.telemetry.addData("arm angle", armAngle);
        myOpMode.telemetry.addData("servo position", servoPosition );
        return true;
    }

    public void setHome() {
        disableLift();  // Stop any closed loop control
        lastPosition = liftMaster.getCurrentPosition();
        setPower(HOME_POWER);
        myOpMode.sleep(250);

        while (!myOpMode.isStopRequested() && (liftMaster.getCurrentPosition() != lastPosition)){
            lastPosition = liftMaster.getCurrentPosition();
            myOpMode.sleep(100);
        }

        setPower(0);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(50);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTarget(0);
        setHandPosition(HAND_OPEN);
        enableLift();  // Start closed loop control
    }

    public void setPower(double power){
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setTarget(int   target){
        this.target = target;
    }

    public double elevatorEncoderToAngle (int position) {
        return(((position * ENCODER_TO_ANGLEM) + ENCODER_TO_ANGLEC));
    }

    public int elevatorAngleToEncoder (double angle) {
        return((int)((angle-ENCODER_TO_ANGLEC)/ENCODER_TO_ANGLEM));
    }

    public double wristAngleToServo (double angle) {
        return(((angle * WRIST_TO_SERVOM ) + WRIST_TO_SERVOC));
    }


    public int getTarget(){
        return target;
    }

    public void enableLift() {
        liftActive = true;
    }

    public void disableLift() {
        liftActive = false;
    }

    public int getPosition() {
        return liftMaster.getCurrentPosition();
    }

    public void manualControl() {
        if (myOpMode.gamepad1.dpad_up && getPosition() <= 1000) {
            setPower(SLOW_LIFT);
        }
        else if (myOpMode.gamepad1.dpad_down && getPosition() > 0) {
            setPower(SLOW_LOWER);
        }
        else {
            setPower(0);
        }
    }

    public void setWristPosition(double angle) {
        wrist.setPosition(angle);
    }

    public void setHandPosition(double angle) {
        hand.setPosition(angle);
    }
}

