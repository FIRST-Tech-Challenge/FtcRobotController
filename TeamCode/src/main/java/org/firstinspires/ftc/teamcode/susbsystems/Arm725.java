package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.teamcode.Constants.SHOULDER_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.SHOULDER_MIN_HEIGHT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Util;

//TODO 12/30/25 Nathan
// GotoHeight functions
// GotoAngle functions
// not precisely calibrated but seem good enough
// homing is manual but works well.  Homed = horizontal

public class Arm725 {

    /* not currently used
    public static final double PID_P_DEFAULT = 2.5;
    public static final double PID_I_DEFAULT = 0;
    public static final double PID_D_DEFAULT = .1;
    private static final double HOME_ANGLE = 0;
    private static final double HOME_Height = 0;

    // the value in volts the potentiometer reads at stowed position
    private final double POT_STOWED_READING = 0.495;
    // the value in volts the potentiometer reads when the arm points straight ahead
    private final double POT_0DEG_READING = 1.32;
    //private final double DEG_TO_VOLT = (POT_STOWED_READING - POT_0DEG_READING) / STOWED_ANGLE_DEG;

    private static final double PIVOT_TO_WRIST = 14;

    //private Vector2 latestDirection = new Vector2();
        private double pCoef = PID_P_DEFAULT, iCoef = PID_I_DEFAULT, dCoef = PID_D_DEFAULT;
    //desired distance offset from the given target in inches
    //this variable can be used to set the extension distance after setting the arm's rotation

    private PIDController controller;

    public boolean AUTOSTOP = true;
  */
    //Shoulder Constants
    private static final double DEG_TO_TICKS = 8.05; //
    private static final double SHOULDER_RADIUS = 19;


    //Shoulder private variables
    private DcMotorEx motor;
    private boolean shoulder_is_busy = false;
    private boolean homed = false;
    private int targetPosition = 0;
    private double currentPosition;
    private double currentPower = 0;
    private double targetHeight = 0;

    public Arm725(HardwareMap hm) {
        init(hm);
    }
    private void init(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "shouldergobilda");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        ResetEncoder();
    }

    public void GoToHeight(double target_height) {
        double temp;
        targetHeight = target_height;
        temp = target_height / SHOULDER_RADIUS;
        //temp = Math.toRadians(temp);
        temp = Math.asin(temp);
        temp = Math.toDegrees(temp);
        targetPosition = (int) (temp*DEG_TO_TICKS);
        shoulder_is_busy = true;
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        currentPower = 0.4;
    }
    public void GoToAngle(double target_angle) {

        this.targetPosition = (int) ( target_angle*DEG_TO_TICKS);
        shoulder_is_busy = true;
        motor.setTargetPosition((int) targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        currentPower = 1;
    }
    public void GoToEncoderPosition(int ticks) {
        targetPosition = ticks;
        shoulder_is_busy = true;
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        currentPower = 1;
    }
    public double GetPos() {return currentPosition / DEG_TO_TICKS;}
    public double GetHeight() {
        double temp;
        temp = currentPosition / DEG_TO_TICKS;
        temp = Math.toRadians(temp);
        temp = Math.sin(temp);
        return (SHOULDER_RADIUS *temp);
    }
    public double getExtension(){
        double temp;
        temp = currentPosition / DEG_TO_TICKS;
        temp = Math.toRadians(temp);
        temp = Math.cos(temp);
        return (SHOULDER_RADIUS *temp);
    }
    public double GetRawPos() {return currentPosition;}
    public double GetTargetPos() {return (targetPosition) / DEG_TO_TICKS;}
    public double GetRawTargetPos() {return targetPosition;}
    public boolean IsBusy() {return shoulder_is_busy;}
    private void rawSet(double power) {
        motor.setPower(Util.IntClamp(power));
    }
    public void SetPower(double power) {
        shoulder_is_busy = false;
        currentPower = power;
    }
    public void Stop() {
        rawSet(0);
        currentPower = 0;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_is_busy = false;
    }
    public void ResetEncoder() {
        currentPower = 0;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentPosition = motor.getCurrentPosition();
    }
    public void Update() {

        currentPosition = motor.getCurrentPosition();
        if (shoulder_is_busy) {
            if (!motor.isBusy()) {
                shoulder_is_busy = false;
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        //safety checks
        if(homed) {
            if (GetHeight() > SHOULDER_MAX_HEIGHT) {
                if (shoulder_is_busy) {
                    Stop();
                }
                currentPower = Math.min(Math.max(currentPower, 0), -1);
            }
            if (GetHeight() < SHOULDER_MIN_HEIGHT) {
                if (shoulder_is_busy) {
                    Stop();
                }
                currentPower = Math.min(Math.max(currentPower, 0), 1);
            }
        }
        rawSet(currentPower);
    }
    public double GetPower() {return currentPower;}
    public double GetTargetHeight() {return targetHeight;}
    public boolean Homed() {return homed;}
    public void Home() {
        homed = true;
        ResetEncoder();
    }
}

