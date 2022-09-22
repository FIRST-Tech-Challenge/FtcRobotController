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

public class FieldOrientedVelocityControl implements EbotsMotionController {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    ArrayList<MecanumWheel> mecanumWheels = new ArrayList<>();
    double maxWheelVelocity;
    double maxAllowedVelocity;     // between 0-maxWheelVelocity
    double maxAllowedSpinVelocity;     // between 0-maxWheelVelocity to reduce spin power

    // The IMU sensor object
    private EbotsImu ebotsImu;
    private double driverFieldHeadingRad;   // Direction of the drive based on alliance side of field
    private Speed speed = Speed.TELEOP;
    private double translateFieldAngleRad;


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public FieldOrientedVelocityControl(LinearOpMode opMode){
        HardwareMap hardwareMap = opMode.hardwareMap;
        maxWheelVelocity = 2500;
        maxAllowedVelocity = 2500;
        maxAllowedSpinVelocity = 2500;

        ebotsImu = EbotsImu.getInstance(hardwareMap);

        driverFieldHeadingRad = Math.toRadians(AllianceSingleton.getDriverFieldHeadingDeg());

        // Create a list of mecanum wheels and store in mecanumWheels
        // Wheel rollers are either 45 or -45 degrees.  Note which ones are negative with this list
        ArrayList<WheelPosition> positionsWithNegativeAngle = new ArrayList<>(
//                Arrays.asList(WheelPosition.FRONT_RIGHT, WheelPosition.BACK_LEFT)  // O-CONFIG
                Arrays.asList(WheelPosition.FRONT_LEFT, WheelPosition.BACK_RIGHT)  // X-Config
        );

        //Loop through each WheelPosition (e.b. FRONT_LEFT, FRONT_RIGHT)
        for(WheelPosition pos: WheelPosition.values()){
            // get motorName and initialize it
            String motorName = pos.getMotorName();
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
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

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    @Override
    public void handleUserInput(Gamepad gamepad){

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


        //Read in the gamepad inputs
        double forwardInput = -gamepad.left_stick_y * maxAllowedVelocity;  //reversing sign because up on gamepad is negative
        double lateralInput = -gamepad.left_stick_x * maxAllowedVelocity;  //reversing sign because right on gamepad is positive


        // Spin input is either provided from the right stick or calculated from error if right trigger or bumper squeezed
        double spinInput = -gamepad.right_stick_x * maxAllowedSpinVelocity;    //Positive means to spin to the left (counterclockwise (CCW) when looking down on robot)

        // See if override is requested and set flag and target heading
//        boolean spinOverrideActive = false;
//        double targetHeadingDeg = 0;
//        if(gamepad.right_trigger > 0.3){
//            spinOverrideActive = true;
//            targetHeadingDeg = -AllianceSingleton.getDriverFieldHeadingDeg();    // note negative sign
//        } else if(gamepad.right_bumper){
//            spinOverrideActive = true;
//            targetHeadingDeg = 0.0;
//        } else if(gamepad.left_bumper){
//            spinOverrideActive = true;
//            targetHeadingDeg = AllianceSingleton.isBlue() ? 30.0 : -30.0;
//        }
//
//        // If override active, calculate spin signal based on PID coefficients and heading error
//        if (spinOverrideActive){
//            double headingErrorDeg = UtilFuncs.applyAngleBounds(targetHeadingDeg - currentHeadingDeg);
//            // apply PID
//            spinInput = headingErrorDeg * speed.getS_p() * maxAllowedSpinVelocity;
//            // don't over-saturate signal while preserving sign
//            double spinSign = Math.signum(spinInput);
//            spinInput = Math.min(maxAllowedSpinVelocity, Math.abs(spinInput));
//            spinInput = spinInput * spinSign;
//        }


        //  Step 1:  Calculate the magnitude for drive signal (hypotenuse of xDirDrive and yDirDrive signal)
        //  Step 2:  Calculate the translate angle (based on X & Y signals, robot heading is not a consideration)
        //  Step 3:  Calculate the robot driveAngle, which adjusts for robot orientation
        //  Step 4:  Calculate and set attribute calculatedPower for each wheel (doesn't set motor power yet)
        //  Step 5:  If needed, scale all the calculated powers so max value equals maxAllowedPower
        //  Step 6:  Apply slow-mo or scale up if required


        //  Step 1:  Calculate the magnitude for drive signal (hypotenuse of xDirDrive and yDirDrive signal)
        double requestedTranslateVelocity = Math.hypot(forwardInput, lateralInput);

        //  Step 2:  Calculate the translate angle (based on X & Y signals, robot heading is not a consideration)
        translateFieldAngleRad = Math.atan2(lateralInput, forwardInput) + driverFieldHeadingRad;

        //  Step 3:  Calculate the robot driveAngle, which adjusts for robot orientation
        double driveAngleRad = translateFieldAngleRad - Math.toRadians(currentHeadingDeg);


        //  Step 4:  Calculate and set attribute calculatedPower for each wheel (doesn't set motor power yet)
        for(MecanumWheel mecanumWheel: mecanumWheels) {
            // to calculate power, must offset translate angle by wheel roller angle
            double calcAngleRad = driveAngleRad - mecanumWheel.getWheelAngleRad();
            double translateVelocity = requestedTranslateVelocity * Math.cos(calcAngleRad);
            double spinVelocity = mecanumWheel.getWheelPosition().getSpinSign() * spinInput;
            mecanumWheel.setCalculatedVelocity(translateVelocity + spinVelocity);
        }

        //  Step 5:  If needed, scale all the calculated powers so max value equals maxAllowedPower
        // Apply a scale factor if maxAllowedPower is exceeded
        // Note:  this can happen when adding translate and spin components
        double maxCalculatedVelocityMagnitude = getMaxCalculatedVelocityMagnitude();
        if (maxCalculatedVelocityMagnitude> maxAllowedVelocity){
            double scaleFactor = maxAllowedVelocity /maxCalculatedVelocityMagnitude;
            this.applyScaleToCalculatedVelocity(scaleFactor);
        }

        //  Step 6:  Apply slow-mo or scale up if required
        //Get the input for Super Slo-Mo
        double superSloMoInput = 1-gamepad.left_trigger;
        double thresholdValue = 0.85;

        // Apply a final refinement to the drive
        // Either: apply super slo-mo  OR  maximize power when driving fast
        if (superSloMoInput < thresholdValue) {
            superSloMoInput = Math.max(0.3, superSloMoInput);
            applyScaleToCalculatedVelocity(superSloMoInput);
        } else if(gamepad.right_trigger > 0.3){
            applyScaleToCalculatedVelocity(0.5);
        }else if (requestedTranslateVelocity >= (thresholdValue * maxAllowedVelocity) &&
                maxCalculatedVelocityMagnitude < requestedTranslateVelocity){
            // sometimes the calculation doesn't drive as fast as expected
            // for instance, moving forward sets all motors to 0.707 power
            // this conditional scales powers up to equal the requested translate magnitude
            // but only when speed matters, so if requested magnitude is > 0.85 and no slow-mo requested
            double scaleFactor = requestedTranslateVelocity / maxCalculatedVelocityMagnitude;
            this.applyScaleToCalculatedVelocity(scaleFactor);
        }

        // This sets the motor to the calculated power to move robot
        drive();
    }


    @Override
    public void stop() {
        for(MecanumWheel mecanumWheel: mecanumWheels){
            mecanumWheel.getMotor().setVelocity(0.0);
        }
    }

    private double getMaxCalculatedVelocityMagnitude(){
        //Loop through the drive motors and return the max abs value of the calculated drive
        double maxCalculatedVelocityMagnitude=0;
        for(MecanumWheel mecanumWheel: mecanumWheels){
            maxCalculatedVelocityMagnitude = Math.max(maxCalculatedVelocityMagnitude, Math.abs(mecanumWheel.getCalculatedVelocity()));
        }
        return maxCalculatedVelocityMagnitude;
    }

    public void applyScaleToCalculatedVelocity(double scaleFactor){
        //Loop through each driveWheel and scale calculatedDrive
        for(MecanumWheel mecanumWheel: mecanumWheels){
            double newVelocity = mecanumWheel.getCalculatedVelocity() * scaleFactor;
            mecanumWheel.setCalculatedVelocity(newVelocity);
        }
    }

    public void drive(){
        // set the calculated power to each wheel
        for(MecanumWheel mecanumWheel: mecanumWheels){
            mecanumWheel.energizeWithCalculatedVelocity();
        }
    }

    public void addVelocitiesToTelemetry(Telemetry telemetry){
        for (MecanumWheel mecanumWheel: mecanumWheels){
            telemetry.addData(mecanumWheel.getWheelPosition().toString(),mecanumWheel.getMotor().getVelocity());
        }
    }


}
