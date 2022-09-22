package org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsenums.WheelPosition;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.UtilFuncs;

import java.util.ArrayList;
import java.util.Arrays;

public class FieldOrientedDrive implements EbotsMotionController {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    Telemetry telemetry;
    private ArrayList<MecanumWheel> mecanumWheels = new ArrayList<>();
    private double maxAllowedPower;     // between 0-1
    private double spinScaleFactor;     // between 0-1 to reduce spin power
    private double requestedTranslateMagnitude;
    private double translateFieldAngleRad;

    // The IMU sensor object
    private EbotsImu ebotsImu;

    private double currentHeadingDeg;          // current field-heading of the robot
    private double driverFieldHeadingRad;   // Direction of the drive based on alliance side of field
    private Speed speed = Speed.TELEOP;
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public FieldOrientedDrive(LinearOpMode opMode){
        HardwareMap hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        speed = Speed.TELEOP;
        maxAllowedPower = speed.getMaxSpeed();
        spinScaleFactor = speed.getTurnSpeed();

        ebotsImu = EbotsImu.getInstance(hardwareMap);

        driverFieldHeadingRad = Math.toRadians(AllianceSingleton.getDriverFieldHeadingDeg());

        // Create a list of mecanum wheels and store in mecanumWheels
        // Wheel rollers are either 45 or -45 degrees.  Note which ones are negative with this list
        ArrayList<WheelPosition> positionsWithNegativeAngle = new ArrayList<>(
                Arrays.asList(WheelPosition.FRONT_LEFT, WheelPosition.BACK_RIGHT)  // X-Config
        );

        //Loop through each WheelPosition (e.b. FRONT_LEFT, FRONT_RIGHT)
        for(WheelPosition pos: WheelPosition.values()){
            // get motorName and initialize it
            String motorName = pos.getMotorName();
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    public double getCurrentHeadingDeg(){
        return currentHeadingDeg;
    }

    @Override
    public String getName() {
        return this.getClass().getSimpleName();
    }

    public double getZeroHeadingDeg() {
        return ebotsImu.getFieldHeadingWhenInitializedDeg();
    }


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    @Override
    public void stop() {
        for(MecanumWheel mecanumWheel: mecanumWheels){
            mecanumWheel.getMotor().setPower(0.0);
        }
    }

    @Override
    public void handleUserInput(Gamepad gamepad) {
        //  For field-oriented drive, the inputted forward and lateral commands are intepreted as:
        //  Assuming Red Alliance:
        //      FORWARD:  Robot translates Field Coordinate Y+
        //      LATERAL:  Left translated Field Coordinate X-

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

        // force an imu read;
        double currentHeadingDeg = ebotsImu.getCurrentFieldHeadingDeg(true);

        //Read in the gamepad inputs and update current heading
        double forwardInput = -gamepad.left_stick_y;  //reversing sign because up on gamepad is negative
        double lateralInput = -gamepad.left_stick_x;  //reversing sign because right on gamepad is positive

        // Spin input is either provided from the right stick or calculated from error if right trigger or bumper squeezed
        double spinInput = -gamepad.right_stick_x * spinScaleFactor;    //Positive means to spin to the left (counterclockwise (CCW) when looking down on robot)

        // See if override is requested and set flag and target heading
        boolean spinOverrideActive = false;
        double targetHeadingDeg = 0;
        if(gamepad.right_trigger > 0.3){
            spinOverrideActive = true;
            targetHeadingDeg = -AllianceSingleton.getDriverFieldHeadingDeg();    // note negative sign
        } else if(gamepad.right_bumper){
            spinOverrideActive = true;
            targetHeadingDeg = 0.0;
        } else if(gamepad.left_bumper){
            spinOverrideActive = true;
            targetHeadingDeg = AllianceSingleton.isBlue() ? 30.0 : -30.0;
        }

        // If override active, calculate spin signal based on PID coefficients and heading error
        if (spinOverrideActive){
            double headingErrorDeg = UtilFuncs.applyAngleBounds(targetHeadingDeg - currentHeadingDeg);
            // apply PID
            spinInput = headingErrorDeg * speed.getS_p();
            // don't over-saturate signal while preserving sign
            double spinSign = Math.signum(spinInput);
            spinInput = Math.min(spinScaleFactor, Math.abs(spinInput));
            spinInput = spinInput * spinSign;
        }

        //  Step 1:  Calculate the magnitude for drive signal (hypotenuse of xDirDrive and yDirDrive signal)
        //  Step 2:  Calculate the translate angle (based on X & Y signals, robot heading is not a consideration)
        //  Step 3:  Calculate the robot angle, which adjusts for robot orientation
        //  Step 4:  Calculate the motor power and set mecanumWheel attribute calculated power (doesn't set motor power yet)
        //  Step 5:  If needed, scale all the calculated powers so max value equals maxAllowedPower
        //  Step 6:  Apply slow-mo or scale up if required

        //  Step 1:  Calculate the magnitude for drive signal (hypotenuse of xDirDrive and yDirDrive signal)
        requestedTranslateMagnitude = Math.hypot(forwardInput, lateralInput);

        //  Step 2:  Calculate the translate angle (based on X & Y signals, considering driver orientation)
        translateFieldAngleRad = Math.atan2(lateralInput, forwardInput) + driverFieldHeadingRad;

        //  Step 3:  Calculate the robot driveAngle, which adjusts for robot orientation
        double driveAngleRad = translateFieldAngleRad - Math.toRadians(currentHeadingDeg);
        // overflow of angle is OK here, the calculation isn't affected
        //driveAngleRad=Math.toRadians(applyAngleBounds(Math.toDegrees(driveAngleRad)));

        //  Step 4:  Calculate and set attribute calculatedPower for each wheel (doesn't set motor power yet)
        for(MecanumWheel mecanumWheel: mecanumWheels) {
            // to calculate power, must offset translate angle by wheel roller angle
            double calcAngleRad = driveAngleRad - mecanumWheel.getWheelAngleRad();
            double translatePower = requestedTranslateMagnitude * Math.cos(calcAngleRad);
            double spinPower = mecanumWheel.getWheelPosition().getSpinSign() * spinInput;
            mecanumWheel.setCalculatedPower(translatePower + spinPower);
        }

        //  Step 5:  If needed, scale all the calculated powers so max value equals maxAllowedPower
        double maxCalculatedPowerMagnitude = getMaxCalculatedPowerMagnitude();
        if (maxCalculatedPowerMagnitude>maxAllowedPower){
            double scaleFactor = maxAllowedPower/maxCalculatedPowerMagnitude;
            this.applyScaleToCalculatedDrive(scaleFactor);
        }

        //  Step 6:  Apply slow-mo or scale up if required
        //Get the input for Super Slo-Mo
        double superSloMoInput = 1-gamepad.left_trigger;
        double thresholdValue = 0.85;

        // Either: apply super slo-mo  OR  maximize power when driving fast
        if (superSloMoInput < thresholdValue){
            superSloMoInput = Math.max(0.3, superSloMoInput);
            applyScaleToCalculatedDrive(superSloMoInput);
        }else if (requestedTranslateMagnitude >= thresholdValue &&
                maxCalculatedPowerMagnitude < requestedTranslateMagnitude){
            // sometimes the calculation doesn't drive as fast as expected
            // for instance, moving forward sets all motors to 0.707 power
            // this conditional scales powers up to equal the requested translate magnitude
            // but only when speed matters, so if requested magnitude is > 0.85
            double scaleFactor = requestedTranslateMagnitude / maxCalculatedPowerMagnitude;
            this.applyScaleToCalculatedDrive(scaleFactor);
        }

        // This sets the motor to the calculated power to move robot
        drive();

    }

    private double getMaxCalculatedPowerMagnitude(){
        //Loop through the drive motors and return the max abs value of the calculated drive
        double maxCalculatedPowerMagnitude=0;
        for(MecanumWheel mecanumWheel: mecanumWheels){
            maxCalculatedPowerMagnitude = Math.max(maxCalculatedPowerMagnitude, Math.abs(mecanumWheel.getCalculatedPower()));
        }
        return maxCalculatedPowerMagnitude;
    }

    public void applyScaleToCalculatedDrive(double scaleFactor){
        //Loop through each driveWheel and scale calculatedDrive
        for(MecanumWheel mecanumWheel: mecanumWheels){
            double newPower = mecanumWheel.getCalculatedPower() * scaleFactor;
            mecanumWheel.setCalculatedPower(newPower);
        }
    }

    public void drive(){
        // set the calculated power to each wheel
        for(MecanumWheel mecanumWheel: mecanumWheels){
            mecanumWheel.getMotor().setPower(mecanumWheel.getCalculatedPower());
        }
    }

}
