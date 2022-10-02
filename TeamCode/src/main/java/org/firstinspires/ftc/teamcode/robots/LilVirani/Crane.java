package org.firstinspires.ftc.teamcode.robots.LilVirani;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.servoNormalize;
import static org.firstinspires.ftc.teamcode.robots.LilVirani.util.Constants.BULB_SERVO_OPEN;
import static org.firstinspires.ftc.teamcode.robots.LilVirani.util.Constants.BULB_SERVO_CLOSED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDController;

/**
 * Created by 2938061 on 11/10/2017.
 */
@Config
public class Crane {

    //number of ticks per revolution REV HD motor: 2240
    //number of ticks per revolution REV Core    : 288

    DcMotor elbow = null;
    DcMotor extendArm = null;
    Servo bulbServo = null;

    PIDController extendPID;
    public static double kpExtendArm = 0.006; //proportional constant multiplier goodish
    public static double kiExtendArm = 0.0; //integral constant multiplier
    public static double kdExtendArm = 0.0; //derivative constant multiplier
    double extendCorrection = 0.00; //correction to apply to extention motor

    PIDController elbowPID;
    public static double kpElbow = 0.006; //proportional constant multiplier goodish
    public static double kiElbow = 0.0; //integral constant multiplier
    public static double kdElbow = 0.0; //derivative constant multiplier
    double elbowCorrection = 0.00; //correction to apply to elbow motor
    boolean elbowActivePID = true;
    boolean extendArmActivePID = true;


    int ticksPerDegree = 0;
    int ticksPerMeter = 0;


    int elbowPosInternal = 0;
    int elbowPos = 0;
    double elbowPwr = 0;

    int extendArmPosInternal = 0;
    int extendArmPos = 0;
    double extendArmPwr = 0;

    int elbowMax = 0;
    int elbowMin = 0;
    int elbowMaxSafetyOffset = 0;
    public boolean bulbServoOpen = true;


    //constraints
    public int extendMin = 800;
    public int extendMax = 1800;


    public Crane(DcMotor elbow, DcMotor extendArm, Servo bulbServo) {

        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setTargetPosition(elbow.getCurrentPosition());
//        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elbow.setDirection(DcMotorSimple.Direction.REVERSE);

        //extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm.setTargetPosition(extendArm.getCurrentPosition());
//        extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //hook.setDirection(DcMotorSimple.Direction.REVERSE);

        this.elbow = elbow;
        this.extendArm = extendArm;
        this.bulbServo = bulbServo;

        elbowPID = new PIDController(kpElbow, kiElbow, kdElbow);
        elbowPID.setIntegralCutIn(40);
        elbowPID.enableIntegralZeroCrossingReset(false);


    }


    public void update() {

        updateBulbServo();

        if (elbowActivePID)
            movePIDElbow(kpElbow, kiElbow, kdElbow, elbow.getCurrentPosition(), elbowPos);
        else {
            elbowPos = elbow.getCurrentPosition();
        }

        if (extendArmActivePID)
            movePIDExtend(kpExtendArm, kiExtendArm, kdExtendArm, extendArm.getCurrentPosition(), extendArmPos);
        else {
            extendArmPos = extendArm.getCurrentPosition();
        }


    }

    public void movePIDExtend(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //initialization of the PID calculator's output range, target value and multipliers
        extendPID.setOutputRange(-extendArmPwr, extendArmPwr);
        extendPID.setPID(Kp, Ki, Kd);
        extendPID.setSetpoint(targetTicks);
        extendPID.enable();

        //initialization of the PID calculator's input range and current value
        //extendPID.setInputRange(0, 360);
        //extendPID.setContinuous();

        extendPID.setInput(currentTicks);

        //calculates the correction to apply
        extendCorrection = extendPID.performPID();

        //performs the extension with the correction applied
        extendArm.setPower(extendCorrection);
    }

    public void movePIDElbow(double Kp, double Ki, double Kd, double currentTicks, double targetTicks) {

        //initialization of the PID calculator's output range, target value and multipliers
        elbowPID.setOutputRange(-elbowPwr, elbowPwr);
        elbowPID.setPID(Kp, Ki, Kd);
        elbowPID.setSetpoint(targetTicks);
        elbowPID.enable();

        //initialization of the PID calculator's input range and current value
        elbowPID.setInput(currentTicks);

        //calculates the correction to apply
        elbowCorrection = elbowPID.performPID();

        //moves elbow with the correction applied
        elbow.setPower(elbowCorrection);
    }


    public void updateBulbServo() {
        if (bulbServoOpen) {
            bulbServo.setPosition(servoNormalize(BULB_SERVO_OPEN));
            bulbServoOpen = false;
        } else {
            bulbServo.setPosition(servoNormalize(BULB_SERVO_CLOSED));
            bulbServoOpen = true;
        }

    }

    public void openBulbServo() {

        bulbServoOpen = true;
    }

    public void closeBulbServo() {
        bulbServoOpen = false;
    }


    public void setElbowActivePID(boolean isActive) {
        elbowActivePID = isActive;
    }

    public void setExtendABobActivePID(boolean isActive) {
        extendArmActivePID = isActive;
    }


    private void setExtendArmTargetPos(int pos) {
        extendArmPos = Math.min(Math.max(pos, extendMin), extendMax);
    }

    public int getExtendArmTargetPos() {
        return extendArmPos;
    }

    public int getExtendArmCurrentPos() {
        return extendArm.getCurrentPosition();
    }

    public void setExtendArmPwr(double pwr) {
        extendArmPwr = pwr;
    }


    private void setElbowTargetPos(int pos) {
        elbowPos = Math.min(Math.max(pos, elbowMin), elbowMax - elbowMaxSafetyOffset);
    }


    public void setElbowTargetPos(int pos, double speed) {
        setElbowTargetPos(pos);
        setElbowPwr(speed);

    }


    public boolean setElbowTargetAngle(double angleDegrees) {

        setElbowTargetPos((int) (ticksPerDegree * angleDegrees));
        return true;
    }

    public int getElbowTargetPos() {
        return elbowPos;
    }

    public int getElbowCurrentPos() {
        return elbow.getCurrentPosition();
    }

    public double getCurrentAngle() {
        return elbow.getCurrentPosition() / ticksPerDegree;
    }

    public void setExtendABobLengthMeters(double lengthMeters) {
        setExtendArmTargetPos((int) (lengthMeters * ticksPerMeter));
    }

    public double getCurrentLengthInMeters() {
        return (ticksPerMeter) * getExtendArmCurrentPos();
    }

    public void setElbowPwr(double pwr) {
        elbowPwr = pwr;
    }

    public void stopAll() {
        setElbowPwr(0);
        setExtendArmPwr(0);
        setElbowActivePID(false);
        setExtendABobActivePID(false);
        update();

    }

    public void restart(double elbowPwr, double extendABobPwr) {
        setElbowPwr(elbowPwr);
        setExtendArmPwr(extendABobPwr);

    }

    public void resetEncoders() {
        //just encoders - only safe to call if we know collector is in normal starting position
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public boolean extendToPosition(int position, double speed) {
        setExtendArmPwr(speed);
        setExtendArmTargetPos(position);
        if ((Math.abs(getExtendArmCurrentPos() - getExtendArmTargetPos())) < 15) {
            return true;
        }
        return false;
    }

}







