package org.firstinspires.ftc.teamcode.subsystems.drive.bc4h;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BC4HDriveSubsystem extends SubsystemBase {

    public enum motorName {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT

    }
    //Motor in Port 0, Rev Hub 1.
    private DcMotorEx motorFrontLeft = null;
    //Motor in Port 1, Rev Hub 1.
    private DcMotorEx motorFrontRight = null;
    //Motor in Port 2, Rev Hub 1.
    private DcMotorEx motorBackRight = null;
    //Motor in Port 3, Rev Hub 1.
    private DcMotorEx motorBackLeft = null;

    private int REV_ENCODER_CLICKS = 560;
    private double REV_WHEEL_DIAM = 7.5;
    private  double REV_WHEEL_CIRC = (REV_WHEEL_DIAM/2) * Math.PI;
    final double CLICKS_PER_CM = REV_ENCODER_CLICKS / REV_WHEEL_CIRC;;

    private final double zeroPower = 0.0;

    private static final double MAX_SLOWDOWN = 1.0;
    private static final double MIN_SLOWDOWN = 0.5;

    private boolean slowdownFlag = false;
    private double slowdown = MAX_SLOWDOWN;


    public BC4HDriveSubsystem(final HardwareMap hwMap,
                              final String deviceNameFl,
                              final String deviceNameFr,
                              final String deviceNameBl,
                              final String deviceNameBr){
        //Define and initialize drivetrain motors.
        motorFrontLeft = hwMap.get(DcMotorEx.class, deviceNameFl);
        motorFrontRight = hwMap.get(DcMotorEx.class, deviceNameFr);
        motorBackLeft = hwMap.get(DcMotorEx.class, deviceNameBl);
        motorBackRight = hwMap.get(DcMotorEx.class, deviceNameBr);

        setZeroPowerForAll();

    }

    public BC4HDriveSubsystem(final HardwareMap hwMap,
                              final String deviceNameFl,
                              final String deviceNameFr,
                              final String deviceNameBl,
                              final String deviceNameBr, int revEncoderClicks, double revWheelDiam){
        //Define and initialize drivetrain motors.
        motorFrontLeft = hwMap.get(DcMotorEx.class, deviceNameFl);
        motorBackLeft = hwMap.get(DcMotorEx.class, deviceNameBl);
        motorFrontRight = hwMap.get(DcMotorEx.class, deviceNameFr);
        motorBackRight = hwMap.get(DcMotorEx.class, deviceNameBr);

        setRevEncoderClicks(revEncoderClicks);
        setRevWheelDiam(revWheelDiam);

        setZeroPowerForAll();

    }

    public BC4HDriveSubsystem(final HardwareMap hwMap,
                              final String deviceNameFl,
                              final String deviceNameFr,
                              final String deviceNameBl,
                              final String deviceNameBr, int revEncoderClicks, double revWheelDiam,
                              DcMotorEx.Direction flDirection,
                              DcMotorEx.Direction blDirection,
                              DcMotorEx.Direction frDirection,
                              DcMotorEx.Direction brDirection){
        //Define and initialize drivetrain motors.
        motorFrontLeft = hwMap.get(DcMotorEx.class, deviceNameFl);
        motorBackLeft = hwMap.get(DcMotorEx.class, deviceNameBl);
        motorFrontRight = hwMap.get(DcMotorEx.class, deviceNameFr);
        motorBackRight = hwMap.get(DcMotorEx.class, deviceNameBr);

        setRevEncoderClicks(revEncoderClicks);
        setRevWheelDiam(revWheelDiam);

        setMotorDirection(motorName.FRONT_LEFT, flDirection);
        setMotorDirection(motorName.BACK_LEFT, blDirection);
        setMotorDirection(motorName.FRONT_RIGHT, frDirection);
        setMotorDirection(motorName.BACK_RIGHT, brDirection);

        setZeroPowerForAll();

    }

    public BC4HDriveSubsystem(final HardwareMap hwMap,
                              final String deviceNameFl,
                              final String deviceNameFr,
                              final String deviceNameBl,
                              final String deviceNameBr,
                              int revEncoderClicks,
                              double revWheelDiam,
                              DcMotorEx.Direction flDirection,
                              DcMotorEx.Direction frDirection,
                              DcMotorEx.Direction blDirection,
                              DcMotorEx.Direction brDirection,
                              DcMotorEx.RunMode allSameMode){
        //Define and initialize drivetrain motors.
        motorFrontLeft = hwMap.get(DcMotorEx.class, deviceNameFl);

        motorFrontRight = hwMap.get(DcMotorEx.class, deviceNameFr);

        motorBackLeft = hwMap.get(DcMotorEx.class, deviceNameBl);
        motorBackRight = hwMap.get(DcMotorEx.class, deviceNameBr);

        //setRevEncoderClicks(revEncoderClicks);
        //setRevWheelDiam(revWheelDiam);

        setMotorDirection(motorName.FRONT_LEFT, flDirection);
        setMotorDirection(motorName.FRONT_RIGHT, frDirection);
        setMotorDirection(motorName.BACK_LEFT, blDirection);
        setMotorDirection(motorName.BACK_RIGHT, brDirection);

        setZeroPowerForAll();
        setSameModeForAll(allSameMode);

    }

    public void setRevEncoderClicks(int revEncoderClicks){
        REV_ENCODER_CLICKS = revEncoderClicks;
    }

    public void setRevWheelDiam(double revWheelDiam){
        REV_WHEEL_DIAM = revWheelDiam;
        setRevWheelCirc();
    }

    public void setMotorDirection(motorName name, DcMotorEx.Direction direction){

        switch (name){
            case FRONT_LEFT:
                motorFrontLeft.setDirection(direction);
                break;
            case FRONT_RIGHT:
                motorFrontRight.setDirection(direction);
                break;
            case BACK_LEFT:
                motorBackLeft.setDirection(direction);
                break;
            case BACK_RIGHT:
                motorBackRight.setDirection(direction);
                break;
        }
    }

    public void setMotorPower(motorName name, Double power){

        switch (name){
            case FRONT_LEFT:
                motorFrontLeft.setPower(power);
                break;
            case BACK_LEFT:
                motorBackLeft.setPower(power);
                break;
            case FRONT_RIGHT:
                motorFrontRight.setPower(power);
                break;
            case BACK_RIGHT:
                motorBackRight.setPower(power);
                break;
        }
    }

    public void setMode(motorName name, DcMotorEx.RunMode mode){

        switch (name){
            case FRONT_LEFT:
                motorFrontLeft.setMode(mode);
                break;
            case BACK_LEFT:
                motorBackLeft.setMode(mode);
                break;
            case FRONT_RIGHT:
                motorFrontRight.setMode(mode);
                break;
            case BACK_RIGHT:
                motorBackRight.setMode(mode);
                break;
        }
    }

    public void restartEncoders(){
        setMode(motorName.FRONT_LEFT, DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorName.FRONT_RIGHT, DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorName.BACK_LEFT, DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorName.BACK_RIGHT, DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        setMode(motorName.FRONT_LEFT, DcMotorEx.RunMode.RUN_USING_ENCODER);
        setMode(motorName.FRONT_RIGHT, DcMotorEx.RunMode.RUN_USING_ENCODER);
        setMode(motorName.BACK_LEFT, DcMotorEx.RunMode.RUN_USING_ENCODER);
        setMode(motorName.BACK_RIGHT, DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    private void setZeroPowerForAll(){
        setMotorPower(motorName.FRONT_LEFT, zeroPower);
        setMotorPower(motorName.FRONT_RIGHT, zeroPower);
        setMotorPower(motorName.BACK_LEFT, zeroPower);
        setMotorPower(motorName.BACK_RIGHT, zeroPower);

    }

    private void setSameModeForAll(DcMotorEx.RunMode mode){
        setMode(motorName.FRONT_LEFT, mode);
        setMode(motorName.FRONT_RIGHT, mode);
        setMode(motorName.BACK_LEFT, mode);
        setMode(motorName.BACK_RIGHT, mode);

    }

    private void setSamePowerForAll(double power){
        setMotorPower(motorName.FRONT_LEFT, power);
        setMotorPower(motorName.FRONT_RIGHT, power);
        setMotorPower(motorName.BACK_LEFT, power);
        setMotorPower(motorName.BACK_RIGHT, power);

    }

    //sets motor target locations to right and left targets
    public void setTargets(int target1, int target2){

        setTarget(motorName.FRONT_LEFT, target1);
        setTarget(motorName.FRONT_RIGHT, target2);
        setTarget(motorName.BACK_LEFT, target1);
        setTarget(motorName.BACK_RIGHT, target2);
    }

    //sets motor target locations to right and left targets
    public void setTarget(motorName name, int target){
        switch (name){
            case FRONT_LEFT:
                motorFrontLeft.setTargetPosition(target);
                break;
            case FRONT_RIGHT:
                motorFrontRight.setTargetPosition(target);
                break;
            case BACK_LEFT:
                motorBackLeft.setTargetPosition(target);
                break;
            case BACK_RIGHT:
                motorBackRight.setTargetPosition(target);
                break;
        }

    }

    //set motor mode to run To position
    public void setRunToPositionForAll(){
        setSameModeForAll(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setSlowdownFlag(boolean status){
        slowdownFlag = status;
    }
    public boolean getSlowdownFlag(){
        return slowdownFlag;
    }
    public void setSlowdown(double slowdown){
        this.slowdown = slowdown;
    }
    public double getSlowdown(){

        if(getSlowdownFlag()){
            slowdown = MAX_SLOWDOWN;
        }
        else
            slowdown = MIN_SLOWDOWN;

        return  slowdown;
    }
    public boolean motorsBusy(){
        return motorFrontRight.isBusy() && motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy();

    }



    private void setRevWheelCirc(){
        REV_WHEEL_CIRC = REV_WHEEL_DIAM * Math.PI;
    }

}
