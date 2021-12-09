package org.firstinspires.ftc.teamcode.subsystems.drive.bc4h;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BC4HDriveSubsystem extends SubsystemBase {

    public enum motorName {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT

    }
    //Motor in Port 0, Rev Hub 1.
    private DcMotor motorFrontLeft = null;
    //Motor in Port 1, Rev Hub 1.
    private DcMotor motorFrontRight = null;
    //Motor in Port 2, Rev Hub 1.
    private DcMotor motorBackRight = null;
    //Motor in Port 3, Rev Hub 1.
    private DcMotor motorBackLeft = null;

    private int REV_ENCODER_CLICKS;
    private double REV_WHEEL_DIAM;
    private  double REV_WHEEL_CIRC;

    private final double zeroPower = 0.0;

    public BC4HDriveSubsystem(final HardwareMap hwMap, final String deviceNameFl, final String deviceNameFr, final String deviceNameBl, final String deviceNameBr){
        //Define and initialize drivetrain motors.
        motorFrontLeft = hwMap.get(DcMotor.class, deviceNameFl);
        motorFrontRight = hwMap.get(DcMotor.class, deviceNameFr);
        motorBackLeft = hwMap.get(DcMotor.class, deviceNameBl);
        motorBackRight = hwMap.get(DcMotor.class, deviceNameBr);

        setZeroPowerForAll();

        REV_ENCODER_CLICKS = 560;
        REV_WHEEL_DIAM = 7.5;

    }

    public BC4HDriveSubsystem(final HardwareMap hwMap, final String deviceNameFl, final String deviceNameBl,
                              final String deviceNameFr, final String deviceNameBr, int revEncoderClicks, double revWheelDiam){
        //Define and initialize drivetrain motors.
        motorFrontLeft = hwMap.get(DcMotor.class, deviceNameFl);
        motorBackLeft = hwMap.get(DcMotor.class, deviceNameBl);
        motorFrontRight = hwMap.get(DcMotor.class, deviceNameFr);
        motorBackRight = hwMap.get(DcMotor.class, deviceNameBr);

        setRevEncoderClicks(revEncoderClicks);
        setRevWheelDiam(revWheelDiam);

        setZeroPowerForAll();

    }

    public BC4HDriveSubsystem(final HardwareMap hwMap, final String deviceNameFl, final String deviceNameBl,
                              final String deviceNameFr, final String deviceNameBr, int revEncoderClicks, double revWheelDiam,
                              DcMotorSimple.Direction flDirection, DcMotorSimple.Direction blDirection, DcMotorSimple.Direction frDirection, DcMotorSimple.Direction brDirection){
        //Define and initialize drivetrain motors.
        motorFrontLeft = hwMap.get(DcMotor.class, deviceNameFl);
        motorBackLeft = hwMap.get(DcMotor.class, deviceNameBl);
        motorFrontRight = hwMap.get(DcMotor.class, deviceNameFr);
        motorBackRight = hwMap.get(DcMotor.class, deviceNameBr);

        setRevEncoderClicks(revEncoderClicks);
        setRevWheelDiam(revWheelDiam);

        setMotorDirection(motorName.FRONT_LEFT, flDirection);
        setMotorDirection(motorName.BACK_LEFT, blDirection);
        setMotorDirection(motorName.FRONT_RIGHT, frDirection);
        setMotorDirection(motorName.BACK_RIGHT, brDirection);

        setZeroPowerForAll();

    }

    public BC4HDriveSubsystem(final HardwareMap hwMap, final String deviceNameFl, final String deviceNameBl,
                              final String deviceNameFr, final String deviceNameBr, int revEncoderClicks, double revWheelDiam,
                              DcMotorSimple.Direction flDirection, DcMotorSimple.Direction blDirection,
                              DcMotorSimple.Direction frDirection, DcMotorSimple.Direction brDirection, DcMotor.RunMode allSameMode){
        //Define and initialize drivetrain motors.
        motorFrontLeft = hwMap.get(DcMotor.class, deviceNameFl);
        motorBackLeft = hwMap.get(DcMotor.class, deviceNameBl);
        motorFrontRight = hwMap.get(DcMotor.class, deviceNameFr);
        motorBackRight = hwMap.get(DcMotor.class, deviceNameBr);

        setRevEncoderClicks(revEncoderClicks);
        setRevWheelDiam(revWheelDiam);

        setMotorDirection(motorName.FRONT_LEFT, flDirection);
        setMotorDirection(motorName.BACK_LEFT, blDirection);
        setMotorDirection(motorName.FRONT_RIGHT, frDirection);
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

    public void setMotorDirection(motorName name, DcMotorSimple.Direction direction){

        switch (name){
            case FRONT_LEFT:
                motorFrontLeft.setDirection(direction);
            case BACK_LEFT:
                motorBackLeft.setDirection(direction);
            case FRONT_RIGHT:
                motorFrontRight.setDirection(direction);
            case BACK_RIGHT:
                motorBackRight.setDirection(direction);
        }
    }

    public void setMotorPower(motorName name, Double power){

        switch (name){
            case FRONT_LEFT:
                motorFrontLeft.setPower(power);
            case BACK_LEFT:
                motorBackLeft.setPower(power);
            case FRONT_RIGHT:
                motorFrontRight.setPower(power);
            case BACK_RIGHT:
                motorBackRight.setPower(power);
        }
    }

    public void setMode(motorName name, DcMotor.RunMode mode){

        switch (name){
            case FRONT_LEFT:
                motorFrontLeft.setMode(mode);
            case BACK_LEFT:
                motorBackLeft.setMode(mode);
            case FRONT_RIGHT:
                motorFrontRight.setMode(mode);
            case BACK_RIGHT:
                motorBackRight.setMode(mode);
        }
    }

    public void restartEncoders(){
        setMode(motorName.BACK_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorName.FRONT_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorName.FRONT_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorName.BACK_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMode(motorName.BACK_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(motorName.FRONT_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(motorName.FRONT_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(motorName.BACK_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void setZeroPowerForAll(){
        setMotorPower(motorName.FRONT_LEFT, zeroPower);
        setMotorPower(motorName.BACK_LEFT, zeroPower);
        setMotorPower(motorName.FRONT_RIGHT, zeroPower);
        setMotorPower(motorName.BACK_RIGHT, zeroPower);

    }

    private void setSameModeForAll(DcMotor.RunMode mode){
        setMode(motorName.FRONT_LEFT, mode);
        setMode(motorName.BACK_LEFT, mode);
        setMode(motorName.FRONT_RIGHT, mode);
        setMode(motorName.BACK_RIGHT, mode);

    }

    private void setSamePowerForAll(double power){
        setMotorPower(motorName.FRONT_LEFT, power);
        setMotorPower(motorName.BACK_LEFT, power);
        setMotorPower(motorName.FRONT_RIGHT, power);
        setMotorPower(motorName.BACK_RIGHT, power);

    }

    //sets motor target locations to right and left targets
    public void setTargets(int target1, int target2){

        setTarget(motorName.FRONT_LEFT, target1);
        setTarget(motorName.BACK_LEFT, target1);
        setTarget(motorName.FRONT_RIGHT, target2);
        setTarget(motorName.BACK_RIGHT, target2);
    }

    //sets motor target locations to right and left targets
    public void setTarget(motorName name, int target){
        switch (name){
            case FRONT_LEFT:
                motorFrontLeft.setTargetPosition(target);
            case BACK_LEFT:
                motorBackLeft.setTargetPosition(target);
            case FRONT_RIGHT:
                motorFrontRight.setTargetPosition(target);
            case BACK_RIGHT:
                motorBackRight.setTargetPosition(target);
        }

    }

    //set motor mode to run To position
    public void setRunToPositionForAll(){
        setSameModeForAll(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean motorsBusy(){
        return motorFrontRight.isBusy() && motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy();

    }



    private void setRevWheelCirc(){
        REV_WHEEL_CIRC = REV_WHEEL_DIAM * Math.PI;
    }

}
