package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//------------------------------------------------------------------------------
//
// PushBotHardware
//
//--------
// Extends the OpMode class to provide a single hardware access point for the
// Push Bot.
//--------
public class PushBotHardware extends OpMode
{
    //--------------------------------------------------------------------------
    //
    // v_dc_motor_controller_drive
    //
    //--------
    // This class member manages the aspects of the left drive motor.
    //--------
    private DcMotorController v_dc_motor_controller_drive;

    //--------------------------------------------------------------------------
    //
    // v_motor_left_drive
    // v_channel_left_drive
    //
    //--------
    // These class members manage the aspects of the left drive motor.
    //--------
    private DcMotor v_motor_left_drive;
    final int v_channel_left_drive = 1;

    //--------------------------------------------------------------------------
    //
    // v_motor_right_drive
    // v_channel_right_drive
    //
    //--------
    // These class members manage the aspects of the right drive motor.
    //--------
    private DcMotor v_motor_right_drive;
    final int v_channel_right_drive = 2;

    //--------------------------------------------------------------------------
    //
    // v_motor_left_arm
    //
    //--------
    // This class member manages the aspects of the left arm motor.
    //--------
    DcMotor v_motor_left_arm;

    //--------------------------------------------------------------------------
    //
    // v_servo_left_hand
    //
    //--------
    // This class member manages the aspects of the left hand servo.
    //--------
    Servo v_servo_left_hand;

    //--------------------------------------------------------------------------
    //
    // v_servo_right_hand
    //
    //--------
    // This class member manages the aspects of the right hand servo.
    //--------
    Servo v_servo_right_hand;

    //--------------------------------------------------------------------------
    //
    // PushBotHardware
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public PushBotHardware()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotManual::PushBotHardware

    //--------------------------------------------------------------------------
    //
    // init
    //
    //--------
    // Performs any actions that are necessary when the OpMode is enabled.
    //
    // The system calls this member once when the OpMode is enabled.
    //--------
    @Override public void init ()

    {
        //
        // Use the hardwareMap to associate class members to hardware ports.
        //
        // Note that the names of the devices (i.e. arguments to the get method)
        // must match the names specified in the configuration file created by
        // the FTC Robot Controller (Settings-->Configure Robot).
        //

        //
        // Connect the drive wheel motors.
        //
        // The direction of the right motor is reversed, so joystick inputs can
        // be more generically applied.
        //
        v_dc_motor_controller_drive
            = hardwareMap.dcMotorController.get("drive_controller");

        v_motor_left_drive = hardwareMap.dcMotor.get ("left_drive");

        v_motor_right_drive = hardwareMap.dcMotor.get ("right_drive");
        v_motor_right_drive.setDirection (DcMotor.Direction.REVERSE);

        //
        // Connect the arm motor.
        //
        v_motor_left_arm = hardwareMap.dcMotor.get ("left_arm");

        //
        // Connect the servo motors.
        //
        // Indicate the initial position of both the left and right servos.  The
        // hand should be halfway opened/closed.
        //
        double l_hand_position = 0.5;

        v_servo_left_hand = hardwareMap.servo.get ("left_hand");
        v_servo_left_hand.setPosition (l_hand_position);

        v_servo_right_hand = hardwareMap.servo.get ("right_hand");
        v_servo_right_hand.setPosition (l_hand_position);

    } // PushBotHardware::init

    //--------------------------------------------------------------------------
    //
    // start
    //
    //--------
    // Performs any actions that are necessary when the OpMode is enabled.
    //
    // The system calls this member once when the OpMode is enabled.
    //--------
    @Override public void start ()

    {
        //
        // Only actions that are common to all Op-Modes (i.e. both automatic and
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //

    } // PushBotHardware::start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    //--------
    // Performs any actions that are necessary while the OpMode is running.
    //
    // The system calls this member repeatedly while the OpMode is running.
    //--------
    @Override public void loop ()

    {
        //
        // Only actions that are common to all OpModes (i.e. both auto and\
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //

    } // PushBotHardware::loop

    //--------------------------------------------------------------------------
    //
    // stop
    //
    //--------
    // Performs any actions that are necessary when the OpMode is disabled.
    //
    // The system calls this member once when the OpMode is disabled.
    //--------
    @Override public void stop ()
    {
        //
        // Nothing needs to be done for this OpMode.
        //

    } // PushBotHardware::stop

    //--------------------------------------------------------------------------
    //
    // scale_motor_power
    //
    //--------
    // Scale the joystick input using a nonlinear algorithm.
    //--------
    double scale_motor_power (double p_power)
    {
        //
        // Assume no scaling.
        //
        double l_scale = 0.0f;

        //
        // Ensure the values are legal.
        //
        double l_power = Range.clip (p_power, -1, 1);

        double[] l_array =
            { 0.00, 0.05, 0.09, 0.10, 0.12
            , 0.15, 0.18, 0.24, 0.30, 0.36
            , 0.43, 0.50, 0.60, 0.72, 0.85
            , 1.00, 1.00
            };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int) (l_power * 16.0);
        if (l_index < 0)
        {
            l_index = -l_index;
        }
        else if (l_index > 16)
        {
            l_index = 16;
        }

        if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
        else
        {
            l_scale = l_array[l_index];
        }

        return l_scale;

    } // PushBotManual::scale_motor_power

    //--------------------------------------------------------------------------
    //
    // a_left_drive_power
    //
    //--------
    // Access the left drive motor's power level.
    //--------
    double a_left_drive_power ()
    {
        return v_motor_left_drive.getPower ();

    } // PushBotManual::a_left_drive_power

    //--------------------------------------------------------------------------
    //
    // a_right_drive_power
    //
    //--------
    // Access the right drive motor's power level.
    //--------
    double a_right_drive_power ()
    {
        return v_motor_right_drive.getPower ();

    } // PushBotManual::a_right_drive_power

    //--------------------------------------------------------------------------
    //
    // set_drive_power
    //
    //--------
    // Scale the joystick input using a nonlinear algorithm.
    //--------
    void set_drive_power (double p_left_power, double p_right_power)
    {
        v_motor_left_drive.setPower (p_left_power);
        v_motor_right_drive.setPower (p_right_power);

    } // PushBotManual::set_drive_power

    //--------------------------------------------------------------------------
    //
    // run_using_encoders
    //
    /**
     * Sets both drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_using_encoders ()

    {
        DcMotorController.RunMode l_mode
            = v_dc_motor_controller_drive.getMotorChannelMode
                ( v_channel_left_drive
                );
        if (l_mode == DcMotorController.RunMode.RESET_ENCODERS)
        {
            v_dc_motor_controller_drive.setMotorChannelMode
                ( v_channel_left_drive
                , DcMotorController.RunMode.RUN_USING_ENCODERS
                );
        }

        l_mode = v_dc_motor_controller_drive.getMotorChannelMode
            ( v_channel_right_drive
            );
        if (l_mode == DcMotorController.RunMode.RESET_ENCODERS)
        {
            v_dc_motor_controller_drive.setMotorChannelMode
                ( v_channel_right_drive
                , DcMotorController.RunMode.RUN_USING_ENCODERS
                );
        }

    } // PushBotAuto::run_using_encoders

    //--------------------------------------------------------------------------
    //
    // reset_drive_encoders
    //
    /**
     * Resets both drive wheel encoders.
     */
    public void reset_drive_encoders ()

    {
        //
        // Reset the motor encoders on the drive wheels.
        //
        v_dc_motor_controller_drive.setMotorChannelMode
            ( v_channel_left_drive
            , DcMotorController.RunMode.RESET_ENCODERS
            );

        v_dc_motor_controller_drive.setMotorChannelMode
            ( v_channel_right_drive
            , DcMotorController.RunMode.RESET_ENCODERS
            );

    } // PushBotAuto::reset_drive_encoders

    //--------------------------------------------------------------------------
    //
    // a_left_encoder_count
    //
    //--------
    // Access the left encoder's count.
    //--------
    int a_left_encoder_count ()
    {
        return v_motor_left_drive.getCurrentPosition ();

    } // PushBotManual::a_left_encoder_count

    //--------------------------------------------------------------------------
    //
    // a_right_encoder_count
    //
    //--------
    // Access the right encoder's count.
    //--------
    int a_right_encoder_count ()
    {
        return v_motor_right_drive.getCurrentPosition ();

    } // PushBotManual::a_right_encoder_count

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reached
    //
    //--------
    // Scale the joystick input using a nonlinear algorithm.
    //--------
    boolean have_drive_encoders_reached
        ( double p_left_count
        , double p_right_count
        )
    {
        //
        // Assume failure.
        //
        boolean l_status = false;

        //
        // Have the encoders reached the specified values?
        //
        // TODO Implement stall code using these variables.
        //
        if ((Math.abs (v_motor_left_drive.getCurrentPosition ()) > p_left_count) &&
            (Math.abs (v_motor_right_drive.getCurrentPosition ()) > p_right_count))
        {
            //
            // Set the status to a positive indication.
            //
            l_status = true;
        }

        //
        // Return the status.
        //
        return l_status;

    } // PushBotManual::have_encoders_reached

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reset
    //
    //--------
    // Scale the joystick input using a nonlinear algorithm.
    //--------
    boolean have_drive_encoders_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_status = false;

        //
        // Have the encoders reached zero?
        //
        if ((a_left_encoder_count () == 0) && (a_right_encoder_count () == 0))
        {
            //
            // Set the status to a positive indication.
            //
            l_status = true;
        }

        //
        // Return the status.
        //
        return l_status;

    } // PushBotManual::have_drive_encoders_reset

    //--------------------------------------------------------------------------
    //
    // a_left_arm_power
    //
    //--------
    // Access the left arm motor's power level.
    //--------
    double a_left_arm_power ()
    {
        return v_motor_left_arm.getPower ();

    } // PushBotManual::a_left_arm_power

    //--------------------------------------------------------------------------
    //
    // a_hand_position
    //
    //--------
    // Access the hand position.
    //--------
    double a_hand_position ()
    {
        return v_servo_left_hand.getPosition ();

    } // PushBotManual::a_hand_position

    //--------------------------------------------------------------------------
    //
    // m_hand_position
    //
    //--------
    // Mutate the hand position.
    //--------
    void m_hand_position (double p_position)
    {
        //
        // Ensure the specifiec value is legal.
        //
        double l_position = Range.clip
            ( p_position
            , Servo.MIN_POSITION
            , Servo.MAX_POSITION
            );

        //
        // Set the value.  The right hand value must be opposite of the left
        // value.
        //
        v_servo_left_hand.setPosition (l_position);
        v_servo_right_hand.setPosition (1.0 - l_position);

    } // PushBotManual::m_hand_position
}
