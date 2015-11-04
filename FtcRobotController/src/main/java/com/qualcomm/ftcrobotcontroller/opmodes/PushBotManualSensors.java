package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotManualSensors
//
/**
 * Provide a basic manual operational mode that uses the left and right
 * drive motors, left arm motor, servo motors and gamepad input from two
 * gamepads for the Push Bot.  This manual op-mode use the touch sensors and
 * gamepad controls to drive the arm to a predefined position (i.e. the location
 * of the touch sensor).
 *
 * @author SSI Robotics
 * @version 2015-08-25-14-40
 */
public class PushBotManualSensors extends PushBotTelemetrySensors

{
    //--------------------------------------------------------------------------
    //
    // PushBotManualSensors
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotManualSensors ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotManualSensors

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad and sensor input to
     * transition between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
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
        // Does the user want the arm to rise until the touch sensor is
        // triggered (i.e. is the Y button on gamepad 2 being held down)?
        //
        // Realize that the button may be depressed for multiple loops - the
        // human hand can only react so fast...this loop will be called multiple
        // times before the button is released.
        //
        if (gamepad2.y)
        {
            //
            // If the button has been pressed from a previous iteration, then
            // do not set power to the arm once the touch sensor has been
            // triggered.
            //
            if (!v_raise_arm_automatically)
            {
                v_raise_arm_automatically = true;
            }
        }

        //
        // Has the user commanded the arm to be raised until the touch sensor
        // has been pressed?
        //
        float l_gp2_left_stick_y = -gamepad2.left_stick_y;
        float l_arm_command = 0.0f;
        if (v_raise_arm_automatically)
        {
            //
            // Has the touch sensor been triggered? Or has the user cancelled
            // the operation with the joystick?
            //
            l_arm_command = 1.0f;
            if ((is_touch_sensor_pressed ()) ||
                (Math.abs (l_gp2_left_stick_y) > 0.8)
                )
            {
                //
                // Stop moving the arm.
                //
                l_arm_command = 0.0f;
                v_raise_arm_automatically = false;
            }
        }
        //
        // The user has not commanded the use of the touch sensor.  Apply power
        // to the arm motor according to the joystick value.
        //
        else
        {
            v_raise_arm_automatically = false;

            l_arm_command = (float)scale_motor_power (l_gp2_left_stick_y);
        }
        m_left_arm_power (l_arm_command);

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
        telemetry.addData ("18", "Raise Arm: " + v_raise_arm_automatically);
        telemetry.addData ("19", "Left arm command: " + l_arm_command);

    } // loop

    //--------------------------------------------------------------------------
    //
    // v_raise_arm_automatically
    //
    //--------
    // This class member remembers whether the 'Y' button has been pressed AND
    // released.  The arm is only driven when the button has been pressed AND
    // released to avoid the phenomena where a humans hand holds the button for
    // multiple iterations of the loop method.
    //--------
    private boolean v_raise_arm_automatically = false;

} // PushBotManualSensors
