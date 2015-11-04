package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotTouchEvent
//
/**
 * Provide a basic autonomous operational mode that demonstrates the use of an
 * touch sensor to control the arm using a state machine for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-30-11-45
 */
public class PushBotTouchEvent extends PushBotTelemetrySensors

{
    //--------------------------------------------------------------------------
    //
    // PushBotTouchEvent
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotTouchEvent ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotTouchEvent

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
        // NOTE: The touch sensor controls the WHEELS in this op-mode.  The main
        // use of the touch sensor in the other PushBot[...]Sensor classes is to
        // operate the arm.  This method operates the DRIVE WHEELS.
        //

        //
        // If a touch sensor has been detected, then set the power level to
        // zero.
        //
        if (is_touch_sensor_pressed ())
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

} // PushBotTouchEvent
