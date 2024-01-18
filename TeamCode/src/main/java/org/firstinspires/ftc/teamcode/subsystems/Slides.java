package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Slides extends SubsystemBase {

    public enum SlidePos {
        DOWN(0),
        LOW(150), //way too high
        MED(300),
        HIGH(450);

        public int position;

        SlidePos(int position) {
            this.position = position;
        }
    }

    private DcMotorEx lMotor, rMotor;
    private VoltageSensor voltageSensor;
    private double voltageComp = 1.0;

    public Slides(HardwareMap hardwareMap) {
        lMotor = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        rMotor = hardwareMap.get(DcMotorEx.class, "Right Slide");

        lMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageComp = 12.0 / voltageSensor.getVoltage();
    }

    @Override
    public void periodic() {}

    public void slidePower(double power) {
        lMotor.setPower(power);
        rMotor.setPower(-power);
    }

    public void end() {
        slidePower(0);
    }

    public double getSlidePos() {
        return lMotor.getCurrentPosition();
    }

    public double getSlideVel() {
        return lMotor.getVelocity();
    }

    private static final int UPPER_LIMIT_POSITION = 1500;
    private static final int LOWER_LIMIT_POSITION = 0;

    public boolean atUpper() {
        return getSlidePos() > UPPER_LIMIT_POSITION;
    }

    public boolean atLower() {
        return getSlidePos() < LOWER_LIMIT_POSITION;
    }

    public void resetSlidePos() {
        lMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }



    public double getVoltageComp() {
        return voltageComp;
    }

    public void goToPosition(SlidePos position) {
        lMotor.setTargetPosition(position.position);
        lMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rMotor.setTargetPosition(-position.position);
        rMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slidePower(0.1);
    }
    public void switchMaxMin(){
        if(this.atUpper() == true){
            lMotor.setTargetPosition(LOWER_LIMIT_POSITION);
            lMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slidePower(1.0);
        }
        else if(this.atLower() == true){
            lMotor.setTargetPosition(UPPER_LIMIT_POSITION);
            lMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slidePower(1.0);
        }
        else{
            lMotor.setTargetPosition(LOWER_LIMIT_POSITION);
            lMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slidePower(1.0);
        }
    }
//    public static void goToMax(){
//        lMotor.setTargetPosition(UPPER_LIMIT_POSITION);
//        lMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        slidePower(1.0);
//    }

    public boolean isBusy() {
        return lMotor.isBusy();
    }
}
