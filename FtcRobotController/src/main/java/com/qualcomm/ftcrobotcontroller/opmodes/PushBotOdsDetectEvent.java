package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotOdsDetectEvent
//
/**
 * Provide a basic autonomous operational mode that demonstrates the use of an
 * optical distance sensor to detect a line implemented using a state machine
 * for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-30-11-45
 */
public class PushBotOdsDetectEvent extends PushBotTelemetrySensors

{
    //--------------------------------------------------------------------------
    //
    // PushBotOdsDetectEvent
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotOdsDetectEvent ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotOdsDetectEvent

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
        // If a white line has been detected, then set the power level to zero.
        //
        if (a_ods_white_tape_detected ())
        {
            set_drive_power (0.0, 0.0);
        }
        //
        // Else a white line has not been detected, so set the power level to
        // full forward.
        //
        else
        {
            set_drive_power (1.0, 1.0);
        }

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry

    } // loop

} // PushBotOdsDetectEvent
