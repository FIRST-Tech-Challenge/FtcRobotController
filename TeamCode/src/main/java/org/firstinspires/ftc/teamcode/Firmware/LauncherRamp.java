package org.firstinspires.ftc.teamcode.Firmware.Systems;

// Written by Tobin

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Resources.ServoPlus;

/**
 * Firmware to control the ramp of the launcher. Uses the ServoPlus resource to ensure easy logic.
 * TODO: Test and double check all of the constants, they were guessed based on the CAD and the specs of a Gobilda Torque Servo as found here https://www.gobilda.com/2000-series-dual-mode-servo-25-2-torque on 11/6/2025 - Tobin
 */
@Config
public class LauncherRamp {

    // servo instance
    private ServoPlus rampServo = null;


    //TODO: as said on line 13, All of these values have been guesstimated by Tobin based on the CAD models.

    // the full range of motion of the servo - note that this is usually a little different than what it is on the spec sheet, double check values.
    public static double servoROM = 288.5; // degrees - Based from the specs of a gobilda torque servo - need to identify correct range.

    // The range of motion the ramp can move, from completely retracted to farthest mechanical limit
//    Took radius from pivot point to end of ramp and the circumference segment of the movable ramp segment. Took that ratio of full circumference and part to find ROM
    public static double rampROM = 41.5; // degrees guessed based on CAD.
    // The gear ratio between the launcher hood and the servo gear - Provided by the chief engineer in training who designed this part on hardware.
    public static double servoToRampRatio = (double) 1 / 6;

    // The minimum angle of the ramp up from Horizontal
    // measured from CAD - eyeballed
    public static double minRampAngle = 37.5;

    // The maximum angle of the ramp up from Horizontal used previous values
    private final double MAX_RAMP_ANGLE = minRampAngle + rampROM;
    public static double timeout = 0.2;
    private final double WIGGLE_TOLERANCE = 8;
    private ElapsedTime movementTimeout;

    /**
     * Construct function for this class, gets the servo reference
     * @param hardwareMap used to get the hardware reference for the ramp servo
     */
    public LauncherRamp(HardwareMap hardwareMap){
        // All values are passed to the servo assuming that the servo's zero is the launcher's min.
        rampServo = new ServoPlus(hardwareMap.get(Servo.class, "ramp"),servoROM * servoToRampRatio,rampAngleToServo(minRampAngle), rampAngleToServo(MAX_RAMP_ANGLE));
        movementTimeout = new ElapsedTime();
    }

    /**
     * Sets the launch angle of the servo. will ensure correct range of input and convert angle to servo position.
     * @param angleUpFromHorizontal degrees above Horizontal the tangent line of the hood should be.
     */
    public void setLaunchAngle(double angleUpFromHorizontal){
        angleUpFromHorizontal = Range.clip(angleUpFromHorizontal, minRampAngle, MAX_RAMP_ANGLE-6);

        if (Math.abs(rampServo.getServoPos() - rampAngleToServo(angleUpFromHorizontal)) > WIGGLE_TOLERANCE){
            movementTimeout.reset();
        }
        rampServo.setServoPos(rampAngleToServo(angleUpFromHorizontal));
    }

    /**
     * lowest launch angle, resting position
     */
    public void retract(){
        rampServo.setServoPos(rampAngleToServo(minRampAngle));
    }

    /**
     * Highest launch angle
     */
    public void extendFull(){
        rampServo.setServoPos(rampAngleToServo(MAX_RAMP_ANGLE));
    }

    /**
     * right in between the limits of launch angles
     */
    public void midAngle(){
        rampServo.setServoPos(rampAngleToServo((MAX_RAMP_ANGLE - minRampAngle)/2));
    }

    /**
     * converts degrees over horizontal to something the servo can use by subtracting the offset that is the ramp's min angle.
     * @param angle degrees over horizontal
     * @return 1/6 the angle the servo needs to be at to align hood to angle.
     */
    private double rampAngleToServo(double angle){
        return angle - minRampAngle;
    }
    public boolean isReady(){
        return movementTimeout.seconds() >= timeout;
    }
}