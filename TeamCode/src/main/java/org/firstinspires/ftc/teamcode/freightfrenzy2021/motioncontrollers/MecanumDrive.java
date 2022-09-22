package org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotsenums.WheelPosition;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.EbotsAutonState;
import org.firstinspires.ftc.teamcode.ultimategoal2020.DriveWheel;

import java.util.ArrayList;
import java.util.Arrays;

public class MecanumDrive implements EbotsMotionController {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    ArrayList<MecanumWheel> mecanumWheels = new ArrayList<>();
    double maxAllowedPower;     // between 0-1
    double spinScaleFactor;     // between 0-1 to reduce spin power

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public MecanumDrive(LinearOpMode opMode){
        HardwareMap hardwareMap = opMode.hardwareMap;
        maxAllowedPower = 1.0;
        spinScaleFactor = 1.0;

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

        //Read in the gamepad inputs
        double forwardInput = -gamepad.left_stick_y;  //reversing sign because up on gamepad is negative
        double lateralInput = -gamepad.left_stick_x;  //reversing sign because right on gamepad is positive
        double spinInput = -gamepad.right_stick_x * spinScaleFactor;    //Positive means to spin to the left (counterclockwise (CCW) when looking down on robot)

        //  Step 1:  Calculate the magnitude for drive signal (hypotenuse of xDirDrive and yDirDrive signal)
        //  Step 2:  Calculate the translate angle (based on X & Y signals, robot heading is not a consideration)
        //  Step 3:  Calculate the motor power and set mecanumWheel attribute calculated power (doesn't set motor power yet)
        //  Step 4:  If needed, scale all the calculated powers so max value equals maxAllowedPower

        //  Step 1:  Calculate the magnitude for drive signal (hypotenuse of xDirDrive and yDirDrive signal)
        double requestedTranslateMagnitude = Math.hypot(forwardInput, lateralInput);

        //  Step 2:  Calculate the translate angle (based on X & Y signals, robot heading is not a consideration)
        double translateAngleRad = Math.atan2(lateralInput, forwardInput);

        //  Step 3:  Calculate and set attribute calculatedPower for each wheel (doesn't set motor power yet)
        for(MecanumWheel mecanumWheel: mecanumWheels) {
            // to calculate power, must offset translate angle by wheel roller angle
            double calcAngleRad = translateAngleRad - mecanumWheel.getWheelAngleRad();
            double translatePower = requestedTranslateMagnitude * Math.cos(calcAngleRad);
            double spinPower = mecanumWheel.getWheelPosition().getSpinSign() * spinInput;
            mecanumWheel.setCalculatedPower(translatePower + spinPower);
        }

        //Apply a scale factor if maxAllowedPower is exceeded
        double maxCalculatedPowerMagnitude = getMaxCalculatedPowerMagnitude();
        if (maxCalculatedPowerMagnitude>maxAllowedPower){
            double scaleFactor = maxAllowedPower/maxCalculatedPowerMagnitude;
            this.applyScaleToCalculatedDrive(scaleFactor);
        }

        //Get the input for Super Slo-Mo
        double superSloMoInput = 1-gamepad.left_trigger;
        double thresholdValue = 0.85;

        // Apply a final refinement to the drive
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


    @Override
    public void stop() {
        for(MecanumWheel mecanumWheel: mecanumWheels){
            mecanumWheel.getMotor().setPower(0.0);
        }
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

    public void addVelocitiesToTelemetry(Telemetry telemetry){
        for (MecanumWheel mecanumWheel: mecanumWheels){
            telemetry.addData(mecanumWheel.getWheelPosition().toString(),mecanumWheel.getMotor().getVelocity());
        }
    }


}
