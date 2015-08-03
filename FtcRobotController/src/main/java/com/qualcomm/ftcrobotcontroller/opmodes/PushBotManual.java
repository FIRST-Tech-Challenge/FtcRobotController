package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotManual
//
/**
 * Extends the PushBotTelemetry and PushBotHardware classes to provide a basic
 * manual operational mode for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
public class PushBotManual extends PushBotTelemetry

{
    //--------------------------------------------------------------------------
    //
    // PushBotManual
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public PushBotManual ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotManual::PushBotManual

    //--------------------------------------------------------------------------
    //
    // loop
    //
    //--------
    // Initializes the class.
    //
    // The system calls this member repeatedly while the OpMode is running.
    //--------
    @Override public void loop ()

    {
        //----------------------------------------------------------------------
        //
        // DC Motors
        //
        // Obtain the current values of the joystick controllers.
        //
        // Note that x and y equal -1 when the joystick is pushed all of the way
        // forward (i.e. away from the human holder's body).
        //
        // The clip method guarantees the value never exceeds the range +-1.
        //
        // The DC motors are scaled to make it easier to control them at slower
        // speeds.
        //
        // The setPower methods write the motor power values to the DcMotor
        // class, but the power levels aren't applied until this method ends.
        //

        //
        // Manage the drive wheel motors.
        //
        float l_gp1_left_stick_y = -gamepad1.left_stick_y;
        float l_left_drive_power
            = (float)scale_motor_power (l_gp1_left_stick_y);

        float l_gp1_right_stick_y = -gamepad1.right_stick_y;
        float l_right_drive_power
            = (float)scale_motor_power (l_gp1_right_stick_y);

        set_drive_power (l_left_drive_power, l_right_drive_power);

        //
        // Manage the arm motor.
        //
        float l_gp2_left_stick_y = -gamepad2.left_stick_y;
        float l_left_arm_power = (float)scale_motor_power (l_gp2_left_stick_y);
        v_motor_left_arm.setPower (l_left_arm_power);

        //----------------------------------------------------------------------
        //
        // Servo Motors
        //
        // Obtain the current values of the gamepad 'x' and 'b' buttons.
        //
        // Note that x and b buttons have boolean values of true and false.
        //
        // The clip method guarantees the value never exceeds the allowable range of
        // [0,1].
        //
        // The setPosition methods write the motor power values to the Servo
        // class, but the positions aren't applied until this method ends.
        //
        if (gamepad2.x)
        {
            m_hand_position (a_hand_position () + 0.05);
        }
        else if (gamepad2.b)
        {
            m_hand_position (a_hand_position () - 0.05);
        }

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry
        telemetry.addData ("10", "GP1 Left: " + l_gp1_left_stick_y);
        telemetry.addData ("11", "GP1 Right: " + l_gp1_right_stick_y);
        telemetry.addData ("12", "GP2 Left: " + l_gp2_left_stick_y);
        telemetry.addData ("13", "GP2 X: " + gamepad2.x);
        telemetry.addData ("14", "GP2 Y: " + gamepad2.b);

    } // PushBotManual::loop

} // PushBotManual
