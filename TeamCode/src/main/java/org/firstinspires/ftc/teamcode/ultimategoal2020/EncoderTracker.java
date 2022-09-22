package org.firstinspires.ftc.teamcode.ultimategoal2020;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ebotsenums.EncoderCalibration;
import org.firstinspires.ftc.teamcode.ebotsenums.EncoderModel;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotOrientation;

import java.util.ArrayList;
import java.util.Formatter;

import static java.lang.String.format;

public class EncoderTracker {
    /**
     *   CLASS:     EncoderTracker
     *   INTENT:    EncoderTracker records and computes the differences in encoder positions
     *              and translates them back into x,y (and maybe heading eventually)
     *              Note: This handles use cases for:
     *                  --2 encoders or 3 encoders
     *                  --Real or Virtual encoders
     */


    /***************************************************************88
    //******    CLASS VARIABLES
    //***************************************************************88*/

    private DcMotorEx motor;                      //motor that the encoder is attached to
    private RobotOrientation robotOrientation;  //orientation of the encoder relative to robot reference frame
    //  FORWARD
    //  LATERAL -> Left is positive

    private int currentClicks;              //  Number of clicks for current iteration
    private int newReading;                 //  Variable for storing new encoder click reading during control cycle
                                            //  This is compared to currentClicks to determine movement

    private double spinRadius;              //  Distance from encoder to robot's spin center
    private double calculatedSpinRadius;    //  Calculated and set during calibration procedure (see StateMoveForCalibration)

    private int cumulativeClicks;                //  Cumulative Number of Clicks
    private double cumulativeDistance;               //  Total Distance traveled in inches
    private boolean isVirtual;                  //Determines whether Encoder is Real or Virtual (Virtual encoder can be used for debugging motion controller logic)
    private double wheelDiameter;
    private double clicksPerInch;   //  Clicks per inch of travel

    /**
     */

    private SpinBehavior spinBehavior;          //
    private ClickDirection clickDirection;
    private EncoderModel encoderModel;
    private EncoderCalibration encoderCalibration = null;

    private boolean debugOn = false;
    private final String logTag = "EBOTS";

    /***************************************************************88
     //******    STATIC VARIABLES
     //****************************************************************/


    /***************************************************************88
     //******    GETTERS
     //****************************************************************/
    public SpinBehavior getSpinBehavior() {
        return spinBehavior;
    }
    public RobotOrientation getRobotOrientation() {
        return robotOrientation;
    }
    public boolean getIsVirtual(){return this.isVirtual;}
    public double getWheelDiameter(){return this.wheelDiameter;}
    public double getCalculatedSpinRadius() {return calculatedSpinRadius; }
    public int getCurrentClicks(){
        return this.currentClicks;
    }
    public double getSpinRadius(){return this.spinRadius;}
    public double getClicksPerInch() {return clicksPerInch;}

    /***************************************************************88
     //******    SETTERS
     //****************************************************************/
    public void setCalculatedSpinRadius(double calculatedSpinRadius) {this.calculatedSpinRadius = calculatedSpinRadius; }
    public void setSpinBehavior(SpinBehavior spinBehaviorIn){this.spinBehavior = spinBehaviorIn; }
    public void setSpinRadius(double radius){
        this.spinRadius = radius;
    }
    public void setWheelDiameter(double diam){
        this.wheelDiameter = diam;
    }
    public void setClickDirection(ClickDirection clickDirection){ this.clickDirection = clickDirection; }
    public void setEncoderCalibration(EncoderCalibration encoderCalibration){
        this.encoderCalibration = encoderCalibration;
        this.applyCalibration();
    }

    private void applyCalibration(){
        // Apply the calibrated values fron EncoderCalibration enum

        boolean debugOn = true;

        // perform null check on encoderCalibration for this EncoderTracker
        if(this.encoderCalibration == null){
            Log.d(logTag, "EncoderTracker::applyCalibration -- No encoderCalibration exists for " + this.toString());
            // Just use the default values
            return;
        }

        if(debugOn){
            Log.d(logTag, "EncoderTracker::applycalibration - Before applyCalibration " + this.toString());
        }
        // Apply the calibrated values for wheelDiameter and spinRadius
        EncoderCalibration encoderCalibration = this.encoderCalibration;
        this.setWheelDiameter(encoderCalibration.getCalibratedWheelDiameter());
        this.setClicksPerInch();
        this.setSpinRadius(encoderCalibration.getCalibratedSpinRadius());
        if(debugOn) Log.d(logTag, "EncoderTracker::applyCalibration -- Calibratiion applied to: " + this.toString());
    }

    /***************************************************************88
    //******    ENUMERATIONS
    //****************************************************************/

    public enum SpinBehavior{
        /**
         * Describes whether the encoder count increases or decreases with positive angle change
         * Note:  Since positive angle is when the robot is spinning CCW when viewed from the top
         *        Encoder wheels on the right side of the robot should increase with angle,
         *        while encoder wheels on the left decrease with angle
         */
        INCREASES_WITH_ANGLE
        , DECREASES_WITH_ANGLE
    }

    public enum ClickDirection{
        /**
         * Describes whether the click count increases or decreases with positive x/y travel (forward/left travel)
         */
        STANDARD,
        REVERSE
    }


    /****************************************************************
    //******    CONSTRUCTORS
    //***************************************************************/

    public EncoderTracker(){
        this.cumulativeDistance = 0.0;
        this.cumulativeClicks = 0;
        this.newReading = 0;
        this.wheelDiameter = 1.875;   // Default is 3.0" for 2019 BOT
        this.spinRadius = 6.0;      // Default is 6.0"
        this.spinBehavior = SpinBehavior.INCREASES_WITH_ANGLE;
        this.clickDirection = ClickDirection.STANDARD;
        this.encoderModel = EncoderModel.REV;
        this.setClicksPerInch();
        this.isVirtual = false;
        this.robotOrientation = RobotOrientation.FORWARD;
        this.motor = null;
    }

    public EncoderTracker(DcMotorEx motor, RobotOrientation robotOrientation, EncoderModel encoderModel){
        this();
        //Set the motor variable and reset the encoder count
        this.setMotorAndZeroEncoder(motor);
        this.robotOrientation = robotOrientation;
        this.encoderModel = encoderModel;
        this.setClicksPerInch();        //must be reset if encoder model changes
    }

    public EncoderTracker(boolean isVirtual, RobotOrientation robotOrientation){
        this();
        this.isVirtual = isVirtual;
        this.robotOrientation = robotOrientation;
    }

    /***************************************************************
    //******    STATIC METHODS
    //****************************************************************/

    public static String printAll(ArrayList<EncoderTracker> encoderTrackers){
        //Prints out the current position of all EncoderTrackers in provided list
        StringBuilder sb = new StringBuilder();
        for(EncoderTracker e: encoderTrackers){
            sb.append(e.robotOrientation.toString());
            sb.append(" ");
            sb.append(e.spinBehavior.toString());
            sb.append(": ");
            sb.append(e.currentClicks);
            sb.append(" | ");
        }
        return sb.toString();
    }

    /***************************************************************88
    //******    GETTERS
    //****************************************************************/


    /***************************************************************88
    //******     SETTERS
    //****************************************************************/


    /***************************************************************
    //******    CLASS METHODS
    //****************************************************************/

    private void setMotorAndZeroEncoder(DcMotorEx motor){
        //During setting of encoders, this makes sure that the encoders are zeroed
        //It first sets the encoder class variable for motor to the passed argument
        //Then reads in the runmode, resets the encoders, and sets the runmode back to orignial value

        this.motor = motor;
        this.zeroEncoder();
    }

    public void reverseClickDirection(){
        this.clickDirection = ClickDirection.REVERSE;
    }

    public void zeroEncoder(){
        boolean debugOn = true;
        if(debugOn) Log.d(logTag, "Entering EncoderTracker::zeroEncoder...");
        if(debugOn){
            Log.d(logTag, "Before zeroing, currentClicks: " +
                    this.currentClicks + " new reading: " + this.newReading +
                    " hardware value: " + this.motor.getCurrentPosition());
        }

        DcMotorEx.RunMode incomingMode = this.motor.getMode();
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(incomingMode);
        this.currentClicks = 0;
        this.newReading = 0;
        if(debugOn){
            Log.d(logTag, "After zeroing, currentClicks: " +
                    this.currentClicks + " new reading: " + this.newReading+
                    " hardware value: " + this.motor.getCurrentPosition());
        }
        //this.newReading=0;
    }

    public void setNewReading(){
        //This should occur as part of a bulk reading operation and should only occur once per control cycle
        // which requires expansion hub cache setting to be set to AUTO or MANUAL

        // Since this is a hardware read, do not perform if isVirtual is true
        if (!this.isVirtual) {
            int appliedSign = (this.clickDirection==ClickDirection.STANDARD) ? 1 : -1;
            this.newReading = this.motor.getCurrentPosition() * appliedSign;
        }
    }


    public int getIncrementalClicks(){
        //Note:  Current clicks is the value at the end of the previous loop
        //       readEncoderValue provides the new value to see how much movement has occurred since

        return (this.newReading - this.currentClicks);
    }

    private void setClicksPerInch(){
        this.clicksPerInch = encoderModel.getClicksPerRevolution() / (wheelDiameter * Math.PI);   //Circumferential wheel distance divided by clicks per revolution
    }

    public void updateEncoderCurrentClicks(){
        boolean debugOn = false;
        String logTag = "EBOTS";

        int oldValue = this.currentClicks;
        this.currentClicks = this.newReading;
        if (debugOn) Log.d(logTag, this.robotOrientation.name() + " encoder was " + oldValue
                + " and is now " + this.currentClicks);
    }

    //*************************************************************************
    //  FOR VIRTUAL ENCODERS
    //*************************************************************************

    public void simulateLoopOutput(EbotsRobot2020 robot, long loopDuration){
        boolean debugOn = true;
        String logTag = "EBOTS";
        if(debugOn) Log.d(logTag, "Entering EncoderTracker.simulateLoopOutput...");

        this.processTranslationLoopOutput(robot,loopDuration);
//        if (debugOn) {
//            StringBuilder sb = new StringBuilder();
//            sb.append("Back in EncoderTrack.simulateLoopOutput\n");
//            sb.append(this.toString());
//            Log.d(logTag, sb.toString());
//        }

        //Then apply rotation
        this.processSpinLoopOutput(robot, loopDuration);
    }

    private void processTranslationLoopOutput(EbotsRobot2020 robot, long loopDuration){
        boolean debugOn = true;
        String logTag = "EBOTS";
        if(debugOn) {
            Log.d(logTag, "Entering EncoderTracker.processTranslationLoopOutput...");
            Log.d(logTag, this.toString());
        }


        DriveCommand driveCommand = robot.getDriveCommand();
        if(debugOn) Log.d(logTag, driveCommand.toString());

        double distance = calculateSimulatedDistance(robot, loopDuration);
        double robotDriveAngleRad = driveCommand.getDriveAngleRad();  //robotDriveAngle uses the robot's reference frame
        double distanceComponent;

        if(this.robotOrientation == RobotOrientation.FORWARD){
            distanceComponent = distance * Math.cos(robotDriveAngleRad);
        } else {
            distanceComponent = distance * Math.sin(robotDriveAngleRad);
        }
        if(debugOn){
            StringBuilder sb = new StringBuilder();
            sb.append("For encoder " + this.robotOrientation);
            sb.append(" and robot angle ");
            Formatter fmt = new Formatter(sb);
            fmt.format("%.2f", Math.toDegrees(robotDriveAngleRad));
            sb.append("° distance component = ");
            fmt.format("%.2f", distanceComponent);
            sb.append(" in");
            Log.d(logTag, sb.toString());
        }
        int translationClicks = (int) Math.round(distanceComponent * clicksPerInch);

        if(debugOn){
            StringBuilder sb = new StringBuilder();
            sb.append("Clicks per Inch: ");
            Formatter fmt = new Formatter(sb);
            fmt.format("%.2f", clicksPerInch);
            sb.append(" Translation Clicks: ");
            sb.append(translationClicks);
            Log.d(logTag, sb.toString());
        }

        //Must be careful that the new
        this.newReading += translationClicks;
        if (debugOn) {
            Log.d(logTag, "Added translation output " + (translationClicks) + " clicks to " + this.getRobotOrientation().name() + " encoder");
            Log.d(logTag, this.toString());
        }
    }

    public double calculateSimulatedDistance(EbotsRobot2020 robot, long timeStepMillis){
        boolean debugOn = true;
        if (debugOn) Log.d(logTag, "Entering EncoderTracker.calculateSimulatedDistance...");

        double driveMagnitude = robot.getDriveCommand().getMagnitude();
        double topSpeed = robot.getTopSpeed();
        double actualSpeed = driveMagnitude * topSpeed;    //Assumes uniform top speed in all directions that is linear with driveMagnitude

        double distance = actualSpeed * (timeStepMillis / 1000.0);

        if (debugOn) {
            StringBuilder sb = new StringBuilder();
            Formatter fmt = new Formatter(sb);
            sb.append("actualSpeed: ");
            fmt.format("%.2f", actualSpeed);
            sb.append(" in/s over timestep ");
            fmt.format("%.3f", (timeStepMillis / 1000.0));
            sb.append(" s for distance of ");
            fmt.format("%.3f", distance);
            sb.append(" in");
            Log.d(logTag, sb.toString());
        }
        return distance;
    }

    private void processSpinLoopOutput(EbotsRobot2020 robot, long loopDuration){
        boolean debugOn = true;
        if(debugOn) Log.d(logTag, "Entering processSpinLoopOutput...");

        double spinDistance = calculateSimulatedRotation(robot, loopDuration);  //Can be negative

        int spinClicks = (int) Math.round(spinDistance * this.clicksPerInch);

        //If SpinBehavior is DecreasesWithAngle then invert the sign
        if(this.spinBehavior == SpinBehavior.DECREASES_WITH_ANGLE) spinClicks *= -1;

        //Note:  must add to the new reading
        this.newReading += spinClicks;

        if (debugOn) {
            Log.d(logTag, "Added Spin Clicks" + (spinClicks) + " to " +
                    this.getRobotOrientation().name() + " encoder");
        }
    }

    public double calculateSimulatedRotation(EbotsRobot2020 robot, long timeStepMillis){
        boolean debugOn = true;
        if(debugOn) Log.d(logTag, "Entering calculateSimulatedRotation...");


        double rotationAngleDeg = robot.estimateHeadingChangeDeg(timeStepMillis);
        double rotationDistance = this.spinRadius * Math.toRadians(rotationAngleDeg);

        if (debugOn) Log.d(logTag, "With spin signal " + format("%.2f", robot.getDriveCommand().getSpin()) +
                " rotation distance of " + format("%.2f", rotationDistance) + " output" +
                " which equates to an angle of " + format("%.2f", rotationAngleDeg));

        return rotationDistance;
    }

    @Override
    public String toString(){
        String outputString;
        outputString =  "Virtual: " + isVirtual + "  Orientation: " + this.robotOrientation.name()
                + " Current Clicks: " + this.currentClicks //+ "= Distance: " + format("%.2f", this.cumulativeDistance)
                + " New Reading: " + this.newReading
                + ", clickDirection: " + this.clickDirection.name()
                + ", spinBehavior: " + this.spinBehavior.name()
                + ", clicksPerInch: " + String.format("%.2f", this.clicksPerInch)
                + ", wheelDiameter: " + String.format("%.2f", this.wheelDiameter);
        return outputString;
    }
}
