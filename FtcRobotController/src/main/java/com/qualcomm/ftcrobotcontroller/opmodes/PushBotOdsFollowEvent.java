package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotOdsFollowEvent
//
/**
 * Provide a basic autonomous operational mode that demonstrates the use of an
 * optical distance sensor to follow a line implemented using a state machine
 * for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-30-11-45
 */
public class PushBotOdsFollowEvent extends PushBotTelemetrySensors

{
    //--------------------------------------------------------------------------
    //
    // PushBotOdsFollowEvent
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotOdsFollowEvent ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotOdsFollowEvent

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        //
        // If a white line has been detected, then turn left.
        //
        if (a_ods_white_tape_detected ())
        {
            set_drive_power (0.0, 0.2);
        }
        //
        // Else a white line has not been detected, so turn right.
        //
        else
        {
            set_drive_power (0.2, 0.0);
        }

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry

    } // loop

} // PushBotOdsFollowEvent
