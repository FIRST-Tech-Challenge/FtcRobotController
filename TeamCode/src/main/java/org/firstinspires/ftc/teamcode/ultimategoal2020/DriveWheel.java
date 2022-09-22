package org.firstinspires.ftc.teamcode.ultimategoal2020;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;

/**
 *   CLASS:     DriveWheel
 *   INTENT:    Object used by a robot that contains configuration information
 *              used in performing drive calculations for mecanum drive
 */
public class DriveWheel {

    /**
     * CLASS VARIABLES
     */
    private double wheelAngleRad;
    private double wheelDiameter;
    private DcMotorEx wheelMotor;
    private double calculatedPower;
    //private RobotSide robotSide;
    private WheelPosition2020 wheelPosition;



    /**
     * CONSTRUCTORS
     */
    public DriveWheel(WheelPosition2020 wheelPosition, HardwareMap hardwareMap){
        this.wheelPosition = wheelPosition;
        this.wheelDiameter = 4;     //Wheel diameter in inches
        this.wheelAngleRad = wheelPosition.getWheelAngleRadEnum();
        this.wheelMotor = initializeDriveMotor(wheelPosition, hardwareMap);
        this.calculatedPower = 0;
        //this.robotSide = wheelPosition.getRobotSideEnum();
    }

    /***************************************************
     * GETTERS
     -------------------------------------------------*/
    public double getCalculatedPower(){
        return this.calculatedPower;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public DcMotorEx getWheelMotor(){
        return wheelMotor;
    }

    public int getEncoderClicks(){
        DcMotorEx motor = getWheelMotor();
        return motor.getCurrentPosition();

    }

    public WheelPosition2020 getWheelPosition(){
        return wheelPosition;
    }

    /***************************************************
     * SETTERS
     **************************************************/



    /***************************************************
     * INSTANCE METHODS
     ****************************************************/
    public DcMotorEx initializeDriveMotor(WheelPosition2020 wheelPosition, HardwareMap hardwareMap){
        String motorName = wheelPosition.getMotorName();
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, motorName);

        //Reverse the directions if on the Right side (Uses RobotSide Enumeration)
        if(wheelPosition.getRobotSide()==RobotSide.RIGHT){
            myMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        //Reset encoders and set run mode
        myMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        myMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        return myMotor;
    }

    public void setCalculatedPower(DriveCommand driveCommand){
        //Start by analyzing the translation component of driveCommand
        double calcAngleRad = driveCommand.getDriveAngleRad() - this.wheelAngleRad;
        this.calculatedPower = driveCommand.getMagnitude() * Math.cos(calcAngleRad);

        //Now apply the spin component of driveCommand
        this.applySpinToCalculatedPower(driveCommand);
    }

    private void applySpinToCalculatedPower(DriveCommand driveCommand) {
        // Apply the spin signal to the wheels using the signs stored with wheelPosition
        double spinPower = this.wheelPosition.getSpinSign() * driveCommand.getSpin();
        calculatedPower += spinPower;      //Update the calculated power (Note: spinPower may be negative)
    }

    public void scaleCalculatedPower(double scaleFactor){
        calculatedPower = calculatedPower * scaleFactor;
    }

    public void setMotorPower(){
        wheelMotor.setPower(calculatedPower);
    }

    public void stopMotor(){
        this.calculatedPower = 0;
        this.setMotorPower();
    }
}
