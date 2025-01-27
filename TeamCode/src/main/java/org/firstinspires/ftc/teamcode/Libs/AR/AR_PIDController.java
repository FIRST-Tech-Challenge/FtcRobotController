package org.firstinspires.ftc.teamcode.Libs.AR;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class creates a PID Controller to use with each joint.
 *
 * Instantiate this class for each JOINT in the ARM object.
 *
 * Creation Date: 11/3/2024
 */

public class AR_PIDController
{
    private PIDController controller;

    // These variables are used to customize the PID Controller for the application. All of these
    // variables are available to be adjusted, in real-time, using FTC Dashboard.
    private double p, i, d;
    private double f;

    // This variable need to be customized for the motor being used. PPR (Pulses Per Revolution) is
    // available from the motor manufacturer.
    //private final double ticksPerDegree = 537.7 / 360;  // For GoBilda 312 RPM Motor
    private final double ticksPerDegree = 5281.1 / 360;  // For GoBilda 30 RPM Motor

    LinearOpMode bot;
    private DcMotor motor;
    private String jointName;

    private int loopCount;  // Used for debugging to count the number of PID loops.

    /**
     * Constructor. Perform the setup of the PID Controller.
     * // ToDo: We are passing in the PID values here but I still think there needs to be some customizations for our particular usage with double joints, built off of each other.
     *
     * @param iBot Object to OpMode so you can access HardwareMap, etc.
     * @param iMotor Motor object that is being used in the joint.
     * @param iJointName Name of Object to OpMode so you can access HardwareMap, etc.
     * @param iP Passed in p value.
     * @param iI Passed in i value
     * @param iD Passed in d value
     * @param iF Passed in f value
     */
    public AR_PIDController(LinearOpMode iBot, DcMotor iMotor, String iJointName, double iP, double iI, double iD, double iF)
    {
        this.bot = iBot;
        this.motor = iMotor;
        this.jointName = iJointName;

        p = iP;
        i = iI;
        d = iD;
        f = iF;

        // Create PID Controller
        controller = new PIDController( p, i, d );

        loopCount = 0; // Set to 0 when initialized
    }

    /**
     * This function takes a target value and moves the joint to that position.
     *
     * @param target Value that the joint should move to.
     * @param iLastState The last state of the Arm.
     */
    public void loop(int target, int iCurrentState, int iLastState ) // Input in degrees
    {
        this.controller.setPID( p, i, d );

        double armPos = 0;

        // Are we working on the first joint or second?
        if( this.jointName.equals("first_joint")) {
            armPos = this.motor.getCurrentPosition( ) + ( AR_Arm.FIRST_JOINT_REST_ANGLE * ticksPerDegree );       // armPos is in Ticks
        } else {
            armPos = this.motor.getCurrentPosition( );    // armPos is in Ticks
        }

        // Original Method
        double pid = this.controller.calculate( armPos, target * ticksPerDegree );  // target is in degrees
        double ff = Math.cos( Math.toRadians( target * ticksPerDegree ) ) * f;  // Here we are passing Ticks to the toRadians function.

        double power = pid + ff;  //pid is based off of ticks,  ff is based off of ticks

        // Determine if there are any special conditions required for power output.
        if ( (this.jointName.equals("first_joint")) &&  // First Joint Only.
                (iLastState == AR_Arm.DEPLOY) &&   // Has to be leaving the DEPLOY state.
                ((armPos / ticksPerDegree) < target - 10)) // Can only be in the initial decent. We need to turn this off for maintaining position.
        {
            power = power * 0.1;       // Throttle power back for arm decent.
        }

        this.motor.setPower( power );

        this.bot.telemetry.addData("Power"," (" + this.jointName + ") " + power ); // Degrees
        this.bot.telemetry.addData("Pos(Ticks)", " (" + this.jointName + ") " + armPos ); // Degrees
        this.bot.telemetry.addData("Pos", " (" + this.jointName + ") " + armPos / ticksPerDegree ); // Degrees
        this.bot.telemetry.addData("Target", " (" + this.jointName + ") " + target );
        this.bot.telemetry.addData("States", + iCurrentState + "(" + iLastState + ")" );

/*        if( target != 0 ) {
            double armPosDegrees = ( armPos / ticksPerDegree );
            Log.i("AR_Experimental", this.jointName + "," + loopCount + "," + armPos + "," + armPosDegrees  + "," + target + "," + pid + "," + ff + "," + power );
        }
*/
        loopCount = loopCount + 1;
    }
}