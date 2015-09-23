package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotIrEvent
//
/**
 * Provide a basic autonomous operational mode that demonstrates the use of an
 * IR seeker implemented using a state machine for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-16-08-41
 */
public class PushBotIrEvent extends PushBotTelemetrySensors

{
    //--------------------------------------------------------------------------
    //
    // PushBotIrEvent
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotIrEvent ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotIrEvent

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
        // When the robot is close to the IR beacon, stop the motors and
        // transition to the next state.
        //
        int l_status = drive_to_ir_beacon ();
        if (l_status == drive_to_ir_beacon_running)
        {
            set_first_message ("Driving to IR beacon.");
        }
        else if (l_status == drive_to_ir_beacon_done)
        {
            set_error_message ("IR beacon is close!");
        }
        else if (l_status == drive_to_ir_beacon_not_detected)
        {
            set_error_message ("IR beacon not detected!");
        }
        else if (l_status == drive_to_ir_beacon_invalid)
        {
            set_error_message ("IR beacon return value is invalid!");
        }

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry

    } // loop

} // PushBotIrEvent
