package org.firstinspires.ftc.teamcode.ultimategoal2020;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;

import java.util.Formatter;

import static java.lang.String.format;

class DriveCommand {
    /***************************************************************
     ******    CLASS VARIABLES
     ***************************************************************/
    //The drive signals are all based on ROBOT coordinate system
    private double magnitude;           //How fast the robot should translate [0-1]
    private double driveAngleRad;       //Direction robot should translate 0-->+X[Robot]   pi/2-->+Y[Robot]
    private double spin;                //Direction and speed of robot spinning [-1 - 1]  positive is CCW

    //Threshold values for sensitivity
    private static final double magnitudeThreshold = 0.1;

    //For Intergrator of the PID controller, keep track if the X, Y, and spin signals are saturated
    //Note:  The X and Y Directions are based on FIELD coordinate system
    private boolean isXSignalSaturated;
    private boolean isYSignalSaturated;
    private boolean isSpinSignalSaturated;

    private final String logTag = "EBOTS";
    private final boolean debugOn = false;

    /***************************************************************
     ******    CONSTRUCTORS
     ***************************************************************/

    public DriveCommand(){
        this.magnitude = 0;
        this.spin = 0;
        this.driveAngleRad = 0;
        //saturation booleans initialized to true to prevent integrator calculation during first loop
        isXSignalSaturated = true;
        isYSignalSaturated = true;
        isSpinSignalSaturated = true;
    }

    public DriveCommand(EbotsRobot2020 robot, Speed speed){
        //  This constructor is used to generate DriveCommands during Auton
        //  Calculates a drive command using input from
        //      -PoseError object
        //      -Speed enumeration (contains PID coefficients)
        //  Note: DriveCommand is relative the ROBOT's coordinate system, while error is measured by FIELD coordinate system
        //        Therefore, 2 angles become important:
        //          *translate angle - Field angle that the robot should travel (regardless of robot heading)
        //          *drive angle     - Angle of motion relative to ROBOT coordinate system
        //  Overall strategy is to define drive magnitude and translation angle via PID constants,
        //          then shift drive angle based on difference between heading and translate angle
        //  Step 1:  Use the poseError object to calculate X & Y signals based on PID coefficients from speed settings
        //  Step 2:  Calculate the spin signal using PID coefficients from speed settings
        //  Step 3:  Set values in the driveCommand object for magnitude, driveAngleRad, and spin based on speed limits

        if(debugOn) Log.d(logTag, "Entering DriveCommand(Robot, Speed) constructor...");

        //  Step 1:  Use the poseError object to calculate X & Y signals based on PID coefficients from speed settings
        //           Note:  These use the FIELD coordinate system, they indicate the direction the robot should move (regardless of robot's heading)
        PoseError2020 poseError2020 = robot.getPoseError();

        double xDirFieldSignal = poseError2020.getXError() * speed.getK_p() + poseError2020.getXErrorSum() * speed.getK_i();
        double yDirFieldSignal = poseError2020.getYError() * speed.getK_p() + poseError2020.getYErrorSum() * speed.getK_i();


        //  Step 2:  Calculate the spin signal using PID coefficients from speed settings
        double spinSignal = robot.getPoseError().getHeadingErrorDeg() * speed.getS_p() + poseError2020.getHeadingErrorDegSum() * speed.getS_i();

        if(debugOn) {
            StringBuilder sb = new StringBuilder();
            sb.append("xDirFieldSignal: ");
            sb.append(xDirFieldSignal);
            sb.append(", ");
            sb.append("yDirFieldSignal: ");
            sb.append(yDirFieldSignal);
            sb.append(", ");
            sb.append("spinSignal: ");
            sb.append(spinSignal);
            Log.d(logTag, sb.toString());
        }

        //  Step 3:  Set values in the driveCommand object for magnitude, driveAngleRad, and spin based on speed limits
        //Read in the regulated speeds for translation and spin
        double peakSpinSpeed = speed.getTurnSpeed();
        double peakTranslateSpeed = speed.getMaxSpeed();

        //  Update the saturation boolean variables -- These are used when updating the integrator term in PoseError
        this.isXSignalSaturated = (Math.abs(xDirFieldSignal) >= peakTranslateSpeed);
        this.isYSignalSaturated = (Math.abs(yDirFieldSignal) >= peakTranslateSpeed);
        this.isSpinSignalSaturated = (Math.abs(spinSignal) >= peakSpinSpeed);

        this.setMagnitudeAndDriveAngle(xDirFieldSignal, yDirFieldSignal, robot.getActualPose().getHeadingRad(), peakTranslateSpeed);
        this.setSpinDrive(spinSignal, peakSpinSpeed);
        this.toString();
    }

    public DriveCommand (Gamepad gamepad){
        //  This constructor is used to generate DriveCommands during Auton
        //  Robot Drive Angle is interpreted as follows:
        //
        //      0 degrees -- forward - (Positive X-Direction)
        //      90 degrees -- left   - (Positive Y-Direction)
        //      180 degrees -- backwards (Negative X-Direction)
        //      -90 degrees -- right    (Negative Y-Direction)
        //
        //  NOTE: This convention follows the right hand rule method, where :
        //      +X --> Forward, +Y is Left, +Z is up
        //   +Spin --> Counter clockwise
        //boolean debugOn = false;
        if(debugOn) Log.d(logTag, "Entering calculateDriveCommandFromGamepad...");


        //Read in the gamepad inputs
        double forwardInput = -gamepad.left_stick_y;  //reversing sign because up on gamepad is negative
        double lateralInput = -gamepad.left_stick_x;  //reversing sign because right on gamepad is positive
        double spinInput = -gamepad.right_stick_x;    //Positive means to spin to the left (counterclockwise (CCW) when looking down on robot)

        double translateMaxSignal = Speed.TELEOP.getMaxSpeed();
        double spinMaxSignal = Speed.TELEOP.getTurnSpeed();

        //todo:  VERIFY the super Slow Mo controls
//        double minAllowed = 0.3;
//        //Get the input for Super Slo-Mo.  If it is less than the minAllowed than use minAllowed
//        double superSloMoInput = Math.max(1-gamepad.left_trigger, minAllowed);
//        translateMaxSignal = Math.min(superSloMoInput, translateMaxSignal);
//        spinMaxSignal = Math.min(superSloMoInput, spinMaxSignal);

        //Set the values for the robot's driveCommand object
        this.setMagnitudeAndDriveAngle(forwardInput, lateralInput, translateMaxSignal);
        this.setSpinDrive(spinInput, spinMaxSignal);
    }



    /***************************************************************
     ******    SIMPLE GETTERS AND SETTERS
     ***************************************************************/
    public double getMagnitude() {
        return magnitude;
    }
    public double getSpin() {
        return spin;
    }
    public double getDriveAngleRad() {
        return driveAngleRad;
    }

    public boolean isSignalSaturated(CsysDirection dir) {
        boolean returnValue = true;
        if(dir == CsysDirection.X) returnValue = isXSignalSaturated;
        else if(dir == CsysDirection.Y) returnValue = isYSignalSaturated;
        else if(dir == CsysDirection.Heading) returnValue = isSpinSignalSaturated;
        return  returnValue;
    }

    //Setters
    public void setMagnitude(double magnitude) {
        this.magnitude = magnitude;
    }
    public void setSpin(double spin) {
        this.spin = spin;
    }
    public void setDriveAngleRad(double driveAngleRad) {
        this.driveAngleRad = driveAngleRad;
    }

    /***************************************************************
     ******    CLASS INSTANCE METHODS
     ***************************************************************/
    public void setMagnitudeAndDriveAngle(double xDirDrive, double yDirDrive, double robotHeadingAngleRad, double translateMaxSignal){
        //  xDirDrive and yDirDrive represent either gamepad inputs or PID component calculations
        //    And whether they reference FIELD or ROBOT coordinate system is dependent on the value for robotHeadingAngleRad
        //    when robotHeadingAngleRad = 0 then it is ROBOT coordinate system
        //  Step 1:  Calculate the magnitude for drive signal (hypotenuse of xDirDrive and yDirDrive signal)
        //  Step 2:  Calculate the translate angle (based on X & Y signals, robot heading is not a consideration)
        //  Step 3:  Calculate the drive angle (based on robot heading, which way robot should move to achieve target position)
        //  Step 4:  Apply the magnitude and driveAngle values considering threshold and speed limits

        //  Step 1:  Calculate the magnitude for drive signal (hypotenuse of xDirDrive and yDirDrive signal)
        double calculatedMagnitude = Math.hypot(xDirDrive, yDirDrive);

        //  Step 2:  Calculate the translate angle (based on X & Y signals, robot heading is not a consideration)
        double translateAngleRad = Math.atan2(yDirDrive, xDirDrive);

        //  Step 3:  Calculate the drive angle (based on robot heading, which way robot should move to achieve target position)
        //           Example:  If robot should move along +X axis, or 0 deg, but is facing +Y axis, or 90 deg,
        //                     then the robot drive angle should point right, or -90 deg direction
        double driveAngleRad = translateAngleRad - robotHeadingAngleRad;

        if(debugOn) {
            StringBuilder sb = new StringBuilder();
            sb.append("Raw Signal Calcs-->");
            Formatter fmt = new Formatter(sb);
            sb.append("calculatedMagnitude: ");
            fmt.format("%.2f",calculatedMagnitude);
            sb.append(", ");
            sb.append("translateAngleDeg: ");
            fmt.format("%.2f",Math.toDegrees(translateAngleRad));
            sb.append("°, ");
            sb.append("driveAngleRad: ");
            fmt.format("%.2f",Math.toDegrees(driveAngleRad));
            sb.append("°");
            Log.d(logTag, sb.toString());
        }

        //  Step 4:  Apply the magnitude and driveAngle values considering threshold and speed limits
        if(calculatedMagnitude > magnitudeThreshold){
            this.magnitude = applyMagnitudeGovernor(calculatedMagnitude, translateMaxSignal);
            this.driveAngleRad = driveAngleRad;
        } else {
            this.magnitude = 0;
            this.driveAngleRad = 0;
        }

        if(debugOn) {
            StringBuilder sb = new StringBuilder();
            sb.append("Apply Speed Limits-->");
            Formatter fmt = new Formatter(sb);
            sb.append("calculatedMagnitude: ");
            fmt.format("%.2f",this.magnitude);
            sb.append(", ");
            sb.append("driveAngleDeg: ");
            fmt.format("%.2f",Math.toDegrees(driveAngleRad));
            sb.append("°");
            Log.d(logTag, sb.toString());
        }
    }

    public void setMagnitudeAndDriveAngle(double xDirDrive, double yDirDrive, double translateMaxSignal) {
        //This is an overload function when no robotHeadingAngleRad is provided
        //This assumes that no adjustment to angle is necessary, or that that robot heading is inline with translate angle
        setMagnitudeAndDriveAngle(xDirDrive, yDirDrive, 0, translateMaxSignal);
    }

    public double applyMagnitudeGovernor(double inputMagnitude, double translateMaxSignal){
        if(debugOn) Log.d(logTag, "Entering applyMagnitudeGovernor...");

        //Step 1:  Calculate the appropriate scale factor to maintain MaxSignal
        double translateScaleFactor = getSignalScaleFactor(inputMagnitude,translateMaxSignal);

        double outputMagnitude = inputMagnitude * translateScaleFactor;
        return outputMagnitude;
    }

    public void setSpinDrive(double spinInput, double spinMaxSignal){
        //spinMaxSignal is the regulated max signal magnitude allowed between [0-1]

        //Step 1: Calculate the necessary scale factor to regulate speeds to those specified
        double spinScale = getSignalScaleFactor(spinInput, spinMaxSignal);
        this.spin = spinInput * spinScale;
    }

    private double getSignalScaleFactor(double inputSignal, double maxSignal){
        if(debugOn) Log.d(logTag, "Entering getSignalScaleFactor...");
        //maxSignal is the regulated max signal magnitude allowed between [0-1]
        if(maxSignal < 0 | maxSignal > 1){
            return 0;
        }

        double scaleFactor = 1.0;     //Assume no scale is required
        if(Math.abs(inputSignal) > maxSignal){
            //If magnitude of spinInput exceeds maxSignal, update scaleFactor (otherwise keep 1 [no scale])
            scaleFactor = Math.abs(maxSignal/inputSignal);       //scale factor can't be negative
        }
        return scaleFactor;
    }

    @Override
    public String toString(){
        StringBuilder outString = new StringBuilder();
        outString.append("Drive Command: (");
        outString.append(format("%.2f", this.magnitude));
        outString.append(" @ ");
        outString.append(format("%.1f",Math.toDegrees(this.driveAngleRad)));
        outString.append("°, ");
        outString.append(format("%.2f",this.spin));
        outString.append(" spin)");

        return outString.toString();
    }

}
