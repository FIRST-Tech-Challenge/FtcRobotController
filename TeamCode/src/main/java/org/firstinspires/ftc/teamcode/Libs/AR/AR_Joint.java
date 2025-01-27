package org.firstinspires.ftc.teamcode.Libs.AR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class create a AR_Joint object that is used to encapsulate all the code used to control and
 * use the 2024-2025 Aerospace Robotics Robot Arm Joint.
 *
 * Instantiate that class for each AR_Joint in the AR_Arm object.
 *
 * Creation Date: 11/3/2024
 */
public class AR_Joint
{
    private LinearOpMode bot;

    // Joint Motor
    private DcMotor jointMotor;

    // PID Controller for AR_Joint
    private AR_PIDController newPID;

    /**
     * Constructor. Prepare a joint instance.
     *
     * @param iBot Handle to the LinearOpMode.
     * @param iJointName text based name of joint motor in robot configuration.
     * @param iP variable of PID controller.
     * @param iI variable of PID controller.
     * @param iD variable of PID controller.
     * @param iF variable for use with feed forward in PID controller.
     *
     */
    public AR_Joint(LinearOpMode iBot, String iJointName, double iP, double iI, double iD, double iF)
    {
        this.bot = iBot;

        // Attach variable to motor hardware and set up.
        this.jointMotor = bot.hardwareMap.dcMotor.get( iJointName );
        this.jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Not needed but left here for future additions if needed.
        this.jointMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // This tells the motor to run with raw power values and not to listen to the built in encoders. We can still get the data we need from the encoders.

        // Instantiate new PID Controller for this joint.
        this.newPID = new AR_PIDController(this.bot, jointMotor, iJointName, iP, iI, iD, iF);
    }

    /**
     * Move joint to the desired target.
     *
     * @param target degree angle to move the joint to.
     */
    public void moveJoint(int target, int iCurrentState, int iLastState)
    {
        this.newPID.loop(target, iCurrentState, iLastState);
    }
}