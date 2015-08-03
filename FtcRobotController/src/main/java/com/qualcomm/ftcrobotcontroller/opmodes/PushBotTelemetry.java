package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotTelemetry
//
/**
 * Extends the PushBotHardware class to provide basic telemetry for the Push
 * Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-02-13-57
 */
public class PushBotTelemetry extends PushBotHardware

{
    //--------------------------------------------------------------------------
    //
    // update_telemetry
    //
    /**
     * Update the telemetry with current values from the base class.
     */
    public void update_telemetry ()

    {
        //
        // Send telemetry data to the driver station.
        //
        telemetry.addData
            ( "01"
            , "Left Drive: "
                + a_left_drive_power ()
                + ", "
                + a_left_encoder_count ()
            );
        telemetry.addData
            ( "02"
            , "Right Drive: "
                + a_right_drive_power ()
                + ", "
                + a_right_encoder_count ()
            );
        telemetry.addData
            ( "03"
            , "Left Arm: " + a_left_arm_power ()
            );
        telemetry.addData
            ( "04"
            , "Hand Position: " + a_hand_position ()
            );
    } // PushBotTelemetry::loop

} // PushBotTelemetry
