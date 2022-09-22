package org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsenums.WheelPosition;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.ebotsutil.PoseError;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

import java.util.ArrayList;
import java.util.Arrays;

// merged into AutonDrive
@Deprecated
public class AutonDriveVelocityControl implements EbotsMotionController {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private ArrayList<MecanumWheel> mecanumWheels = new ArrayList<>();
    private double requestedTranslateVelocity;
    private double translateAngleRad;
    double maxWheelVelocity;    // Property of the robot
    double maxAllowedVelocity;     // between 0-maxWheelVelocity
    double maxAllowedSpinVelocity;     // between 0-maxWheelVelocity to reduce spin power

    private EbotsAutonOpMode autonOpMode;

    private Speed speed;
    private Telemetry telemetry;

    private StopWatch stopWatch;


    // The IMU sensor object
    private EbotsImu ebotsImu; // use singleton

    private PoseError poseError;
    private double xSignal;
    private double ySignal;
    private double spinVelocity;

    private int targetClicks = 0;
    private final int allowableErrorInClicks = 10;

    private String logTag = "EBOTS";
    private int loopCount = 0;
    private final double clicksPerInch = 48.9;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public AutonDriveVelocityControl(EbotsAutonOpMode autonOpMode){
        this.autonOpMode = autonOpMode;
        this.speed = Speed.MEDIUM;
        this.telemetry = autonOpMode.telemetry;
        ebotsImu = EbotsImu.getInstance(autonOpMode.hardwareMap);
        stopWatch = new StopWatch();

        maxWheelVelocity = 2500;
        maxAllowedVelocity = speed.getMaxSpeed() * maxWheelVelocity;
        maxAllowedSpinVelocity = speed.getTurnSpeed() * maxWheelVelocity;

        // Create a list of mecanum wheels and store in mecanumWheels
        // Wheel rollers are either 45 or -45 degrees.  Note which ones are negative with this list
        ArrayList<WheelPosition> positionsWithNegativeAngle = new ArrayList<>(
                Arrays.asList(WheelPosition.FRONT_LEFT, WheelPosition.BACK_RIGHT)  // X-Config
        );

        mecanumWheels.clear();
        //Loop through each WheelPosition (e.b. FRONT_LEFT, FRONT_RIGHT)
        for(WheelPosition pos: WheelPosition.values()){
            // get motorName and initialize it
            String motorName = pos.getMotorName();
            DcMotorEx motor = autonOpMode.hardwareMap.get(DcMotorEx.class, motorName);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  Reverse motor depending on side
            if(pos.getRobotSide() != RobotSide.RIGHT) motor.setDirection(DcMotorSimple.Direction.REVERSE);

            // set the angle of the rollers, modifying sign if needed
            double wheelAngleDeg = 45;
            if (positionsWithNegativeAngle.contains(pos)) wheelAngleDeg = -wheelAngleDeg;

            // create the new wheel and add it to the list
            MecanumWheel mecanumWheel = new MecanumWheel(wheelAngleDeg, pos, motor);
            mecanumWheels.add(mecanumWheel);
        }

    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    @Override
    public String getName() {
        return this.getClass().getSimpleName();
    }

    public Speed getSpeed() {
        return speed;
    }

    public double getTranslateMagnitude(){
        double translateMagnitude = requestedTranslateVelocity;
        translateMagnitude = Math.min(speed.getMaxSpeed(), translateMagnitude);
        return translateMagnitude;
    }

    public double getTranslateAngleRad() {
        return translateAngleRad;
    }

    public double getClicksPerInch() {
        return clicksPerInch;
    }

    public void setSpeed(Speed speed) {
        this.speed = speed;
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public boolean isSignalSaturated(CsysDirection csysDirection){
        boolean isSignalSaturated = true;

        if (csysDirection == CsysDirection.X){
            isSignalSaturated = Math.abs(xSignal) >= speed.getMaxSpeed();

        } else if (csysDirection == CsysDirection.Y){
            isSignalSaturated = Math.abs(ySignal) >= speed.getMaxSpeed();

        } else if (csysDirection == CsysDirection.Heading){
            isSignalSaturated = Math.abs(spinVelocity) >= speed.getTurnSpeed();
        }

        return isSignalSaturated;
    }



    @Override
    public void stop() {
        for(MecanumWheel mecanumWheel: mecanumWheels){
            mecanumWheel.stopVelocity();
        }
    }

    @Override
    public void handleUserInput(Gamepad gamepad) {

    }

    public void calculateDriveFromError(PoseError poseError){

        //      Two angles become important:
        //          *translate angle - Field angle that the robot should travel (regardless of robot heading)
        //          *drive angle     - Angle of motion relative to ROBOT coordinate system
        //
        //      Example: Requested travel is in X direction (0), Robot is facing Y+ (90deg)
        //               translateAngle = 0
        //               driveAngle = 0-90 = -90  so robot drives to the right

        //  Step 1:  Use the poseError object to calculate X & Y signals based on PID coefficients from speed settings
        //  Step 2:  Calculate the spin signal using PID coefficients from speed settings
        //  Step 3:  Set values in the driveCommand object for magnitude, driveAngleRad, and spin based on speed limits

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

        boolean debugOn = (loopCount % 5 == 0);
        String oneDec = "%.1f";

        // force an imu read;
        double currentHeadingDeg = ebotsImu.getCurrentFieldHeadingDeg(true);


        //This calculates the requested drive signal based on motor power.  Must multiply by velocity for velocity control
        xSignal = poseError.getXError() * speed.getK_p() + poseError.getXErrorSum() * speed.getK_i();
        ySignal = poseError.getYError() * speed.getK_p() + poseError.getYErrorSum() * speed.getK_i();
        spinVelocity = poseError.getHeadingErrorDeg() * speed.getS_p() + poseError.getHeadingErrorDegSum() * speed.getS_i();
        spinVelocity *= maxAllowedSpinVelocity;
        // don't over-saturate signal while preserving sign
        double spinSign = Math.signum(spinVelocity);
        spinVelocity = Math.min(maxAllowedSpinVelocity, Math.abs(spinVelocity));
        spinVelocity *= spinSign;

        //  Step 1:  Calculate the magnitude for drive signal (hypotenuse of xDirDrive and yDirDrive signal)
        //  Step 2:  Calculate the translate angle (based on X & Y signals, robot heading is not a consideration)
        //  Step 3:  Calculate the robot angle, which adjusts for robot orientation
        //  Step 4:  Calculate the motor power and set mecanumWheel attribute calculated power (doesn't set motor power yet)
        //  Step 5:  If needed, scale all the calculated powers so max value equals maxAllowedPower

        //  Step 1:  Calculate the magnitude for drive signal (hypotenuse of xDirDrive and yDirDrive signal)
        requestedTranslateVelocity = Math.hypot(xSignal, ySignal) * maxAllowedVelocity;

        //  Step 2:  Calculate the translate angle (based on X & Y signals, in auton, this is field oriented)
        translateAngleRad = Math.atan2(ySignal, xSignal);



        //  Step 3:  Calculate the robot angle, which adjusts for robot orientation
        double driveAngleRad = translateAngleRad - Math.toRadians(currentHeadingDeg);
        // overflow of angle is OK here, the calculation isn't affected
        //driveAngleRad=Math.toRadians(applyAngleBounds(Math.toDegrees(driveAngleRad)));
        if(debugOn) Log.d(logTag, poseError.toString());

        //  Step 4:  Calculate and set attribute calculatedPower for each wheel (doesn't set motor power yet)
        for(MecanumWheel mecanumWheel: mecanumWheels) {
            // to calculate power, must offset translate angle by wheel roller angle
            double calcAngleRad = driveAngleRad - mecanumWheel.getWheelAngleRad();
            double translateVelocity = requestedTranslateVelocity * Math.cos(calcAngleRad);
            double spinPower = mecanumWheel.getWheelPosition().getSpinSign() * spinVelocity;
            if(debugOn){
                Log.d(logTag, "Drive Calc details for " + mecanumWheel.getWheelPosition().toString() +
                        "\ndrive angle: " + String.format(oneDec, driveAngleRad) +
                        "  wheel angle: " + String.format(oneDec, mecanumWheel.getWheelAngleRad()) +
                        "  calcAngleRad: " + String.format(oneDec, calcAngleRad) +
                        "  translateVelocity: " + String.format(oneDec, translateVelocity) +
                        "  spinPower: " + String.format(oneDec, spinPower));
            }
            mecanumWheel.setCalculatedVelocity(translateVelocity + spinPower);
        }

        //  Step 5:  If needed, scale all the calculated powers so max value equals maxAllowedPower
        // Apply a scale factor if maxAllowedPower is exceeded
        // Note:  this can happen when adding translate and spin components
        double maxCalculatedVelocityMagnitude = getMaxCalculatedVelocityMagnitude();
        if (maxCalculatedVelocityMagnitude > maxAllowedVelocity){
            double scaleFactor = maxAllowedVelocity/maxCalculatedVelocityMagnitude;
            this.applyScaleToCalculatedVelocity(scaleFactor);
        }

        // Step 6:  Apply a soft start if early in run
        long softStartDuration = 1000L;
        long currentTime = stopWatch.getElapsedTimeMillis();
        boolean softStartActive = (currentTime < softStartDuration);
        if (softStartActive){
            double scaleFactor = ((double) currentTime / softStartDuration);
            this.applyScaleToCalculatedVelocity(scaleFactor);
        }


        String driveString = "Signals for X / Y / Spin: " + String.format(oneDec, xSignal) +
                " / " + String.format(oneDec, ySignal) +
                " / " + String.format(oneDec, spinVelocity) +
                "\n" + "softStartActive: " + softStartActive +
                "\n" + stopWatch.toString();
        telemetry.addLine(driveString);
        if (debugOn) Log.d(logTag, driveString);
        // This sets the motor to the calculated power to move robot
        driveVelocity();
        loopCount++;

    }

    public void rotateToFieldHeadingFromError(double headingErrorDeg){

        spinVelocity = headingErrorDeg * speed.getS_p();

        // Apply calculated power to each wheel (doesn't move the bot yet)
        for(MecanumWheel mecanumWheel: mecanumWheels) {
            // to calculate power, must offset translate angle by wheel roller angle
            double spinPower = mecanumWheel.getWheelPosition().getSpinSign() * spinVelocity;
            mecanumWheel.setCalculatedPower(spinPower);
        }

        //  If needed, scale all the calculated powers so max value equals maxAllowedPower
        double maxAllowedPower = speed.getMaxSpeed();
        double maxCalculatedPowerMagnitude = getMaxCalculatedVelocityMagnitude();
        if (maxCalculatedPowerMagnitude > maxAllowedPower){
            double scaleFactor = maxAllowedPower/maxCalculatedPowerMagnitude;
            this.applyScaleToCalculatedVelocity(scaleFactor);
        }

        // set the bot in motion
        driveVelocity();

    }

    private double getMaxCalculatedVelocityMagnitude(){
        //Loop through the drive motors and return the max abs value of the calculated drive
        double maxCalculatedVelocityMagnitude=0;
        for(MecanumWheel mecanumWheel: mecanumWheels){
            maxCalculatedVelocityMagnitude = Math.max(maxCalculatedVelocityMagnitude, Math.abs(mecanumWheel.getCalculatedVelocity()));
        }
        return maxCalculatedVelocityMagnitude;
    }

    public void setEncoderTarget(double travelDistanceInches, double travelDirection){
        targetClicks = (int) Math.round(travelDistanceInches * clicksPerInch);

        // must consider the robot's heading when determining encoder targets
        double currentHeadingDeg = ebotsImu.getCurrentFieldHeadingDeg(true);
        double driveAngleRad = translateAngleRad - Math.toRadians(currentHeadingDeg);

        // Set the motor targets
        for(MecanumWheel mecanumWheel: mecanumWheels){
            mecanumWheel.stopVelocity();
            // each wheel will spin forward or backwards proportional with its orientation to travel direction
            double wheelSign = Math.signum(Math.cos(driveAngleRad - mecanumWheel.getWheelAngleRad()));
            // distance each wheel travels is function of wheelFactor orientation and travel distance
            mecanumWheel.getMotor().setTargetPosition((int) (targetClicks * wheelSign));

        }
        Log.d(logTag, "Encoders after setEncoderTarget---------------");
        logAllEncoderClicks();
    }


    public void applyScaleToCalculatedVelocity(double scaleFactor){
        //Loop through each driveWheel and scale calculatedDrive
        for(MecanumWheel mecanumWheel: mecanumWheels){
            double newVelocity = mecanumWheel.getCalculatedVelocity() * scaleFactor;
            mecanumWheel.setCalculatedVelocity(newVelocity);
        }
    }

    public void driveVelocity(){
        // set the calculated power to each wheel
        for(MecanumWheel mecanumWheel: mecanumWheels){
            Log.d(logTag, mecanumWheel.getWheelPosition().toString() + " Target Velocity: " +
                    String.format("%.1f", mecanumWheel.getCalculatedVelocity()));
            mecanumWheel.energizeWithCalculatedVelocity();
        }
    }

    public long calculateTimeLimitMillis(PoseError poseError){
        boolean debugOn = true;
        String oneDec = "%.1f";
        String intFormat = "%d";
        //Find the expected time required to achieve the target pose
        if(debugOn) Log.d(logTag, "Entering calculateTimeLimitMillis...");

        //First, read in required travel distance and spin
        double translateDistance = poseError.getMagnitude();
        if(debugOn) Log.d(logTag, "translateDistance: " + String.format(oneDec, translateDistance));
        double rotationAngleRad = Math.abs(poseError.getHeadingErrorRad());  //Don't allow negative value

        //Take the robot top Speed (in/s) and multiply by Speed object's top speed [0-1]
        double topTranslationSpeed = speed.getMeasuredTranslateSpeed() * speed.getMaxSpeed();
        if(debugOn) Log.d(logTag, "topTranslationSpeed: " + String.format(oneDec, topTranslationSpeed));

        //Take robots top angular speed (rad/s) and multiply by Speed Objects top turn speed[0-1]
        double topSpinSpeed = speed.getMeasuredAngularSpeedRad() * speed.getTurnSpeed();

        long translateTimeMillis = (long) ((translateDistance / topTranslationSpeed)*1000);
        long spinTimeMillis = (long) ((rotationAngleRad / topSpinSpeed)*1000);
        long bufferMillis = 1500L;      //The buffer is a little extra time allotted (maybe should be percentage)

        //The total calculated time is the travel time and spin time and buffer (plus the soft start duration)
        long calculatedTime = (translateTimeMillis + spinTimeMillis + bufferMillis);
        //long calculatedTime = (translateTimeMillis + spinTimeMillis + bufferMillis + softStart.getDurationMillis());

        if(debugOn) Log.d(logTag, "translateTimeMillis: " +  String.format(intFormat, translateTimeMillis) +
                "  spinTimeMillis: " +  String.format(intFormat, spinTimeMillis) +
                "  bufferMillis: " +  String.format(intFormat, bufferMillis) +
                "  Calculated Time: " + String.format(oneDec, (float)(calculatedTime/1000.0)) + " s");
        return (calculatedTime * 5);
    }


    public int getAverageError(){
        double totalError = 0;
        for(MecanumWheel mecanumWheel: mecanumWheels){
            totalError += Math.abs(mecanumWheel.getMotor().getTargetPosition() - mecanumWheel.getMotorClicks());
        }
        return (int) Math.round(totalError / 4);
    }

    public int getAverageClicks(){
        int totalClicks = 0;
        for(MecanumWheel mecanumWheel: mecanumWheels){
            totalClicks += Math.abs(mecanumWheel.getMotorClicks());
        }
        return (int) Math.round(totalClicks / 4.0);
    }


    public boolean isTargetReached(){
        boolean targetReached = false;
        int error = getAverageError();
        if (error <= allowableErrorInClicks){
            targetReached = true;
        }
        return targetReached;
    }


    public void logAllEncoderClicks(){
        Log.d("EBOTS", "Error target: " + String.format("%d", allowableErrorInClicks));
        for(MecanumWheel mecanumWheel: mecanumWheels){
            DcMotorEx motor = mecanumWheel.getMotor();
            Log.d("EBOTS", "Motor Position:" + String.format("%d", motor.getCurrentPosition()) +
                    " Target Position: " + String.format("%d", motor.getTargetPosition()) +
                    " Error: " + String.format("%d", motor.getCurrentPosition() - motor.getTargetPosition()));
        }
    }


}
