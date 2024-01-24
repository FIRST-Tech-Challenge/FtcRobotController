package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private PID pid;
    public Slides(HardwareMap hardwareMap) {
        lMotor = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        rMotor = hardwareMap.get(DcMotorEx.class, "Right Slide");

        lMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pid = new PID(1, 0, 0);

//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        voltageComp = 12.0 / voltageSensor.getVoltage();
    }

    @Override
    public void periodic() {}

    private void setPower(double power) {
        lMotor.setPower(power);
        rMotor.setPower(-power);
    }

    public void setSlidesPower(double power) {
        lMotor.setPower(power);
        rMotor.setPower(-power);
    }

    public void stop() {
        setPower(0.1);
    }

    public double getPos() {
        return lMotor.getCurrentPosition();
    }

//    public double getSlideVel() {
//        return lMotor.getVelocity();
//    }

    public void runToPos(double targetPos) {
        double currentPos = getPos();
        int output = pid.update(currentPos, targetPos);
        lMotor.setTargetPosition(output);
        lMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rMotor.setTargetPosition(-output);
        rMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean atUpper() {
        return getPos() > 600;
    }

    public boolean atLower() {
        return getPos() < 5;
    }

    public void resetSlidePos() {
        lMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

//    public double getVoltageComp() {
//        return voltageComp;
//    }

//    public void goToPosition(SlidePos position) {
//        lMotor.setTargetPosition(position.position);
//        lMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        rMotor.setTargetPosition(-position.position);
//        rMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        setSlidesPower(0.1);
//    }
}
