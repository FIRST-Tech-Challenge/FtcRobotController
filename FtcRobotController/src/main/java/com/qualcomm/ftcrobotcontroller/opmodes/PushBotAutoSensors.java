package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotAutoSensors
//
/**
 * Provide a basic autonomous operational mode that uses the left and right
 * drive motors and associated encoders, the left arm motor and associated touch
 * sensor, IR seeker V3 and optical distance sensor implemented using a state
 * machine for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-13-19-48
 */
public class PushBotAutoSensors extends PushBotTelemetrySensors

{
    //--------------------------------------------------------------------------
    //
    // PushBotAutoSensors
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotAutoSensors ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotAutoSensors

    //--------------------------------------------------------------------------
    //
    // start
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void start ()

    {
        //
        // Call the PushBotHardware (super/base class) start method.
        //
        super.start ();

        //
        // Reset the motor encoders on the drive wheels.
        //
        reset_drive_encoders ();

    } // start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     * The state machine uses a class member and sensor input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        //
        // Update the state machines
        //
        switch (v_state)
        {
        //
        // State 0.
        //
        case 0:
            //
            // Wait for the encoders to reset.  This might take multiple cycles.
            //
            if (have_drive_encoders_reset ())
            {
                //
                // Begin the next state.  Drive forward.
                //
                drive_using_encoders (-1.0f, 1.0f, -2880, 2880);

                //
                // Transition to the next state.
                //
                v_state++;
            }

            break;
        //
        // State 1.
        //
        case 1:
            //
            // Drive forward at full power until the encoders trip.
            //
            // Forward motion is achieved when the first and second parameters
            // are the same and positive.
            //
            // The magnitude of the first and second parameters determine the
            // speed (0.0 is stopped and 1.0 is full).
            //
            // The third and fourth parameters determine how long the wheels
            // turn.
            //
            // When the encoder values have been reached the call resets the
            // encoders, halts the motors, and returns true.
            //
            if (drive_using_encoders (1.0f, 1.0f, 2880, 2880))
            {
                //
                // The drive wheels have reached the specified encoder values,
                // so transition to the next state when this method is called
                // again.
                //
                v_state++;
            }
            break;
        //
        // State 2.
        //
        case 2:
            //
            // Wait for the encoders to reset.  This might take multiple cycles.
            //
            if (have_drive_encoders_reset ())
            {
                //
                // Begin the next state.  Turn left.
                //
                // There is no conditional (if statement) here, because the
                // encoders can't be read until the next cycle
                // (drive_using_encoders makes the run_using_encoders call,
                // which won't be processed until this method exits).
                //
                drive_using_encoders (-1.0f, 1.0f, -2880, 2880);

                //
                // Start the arm state machine.
                //
                v_arm_state = 1;

                //
                // The drive wheels have reached the specified encoder values,
                // so transition to the next state when this method is called
                // again.
                //
                v_state++;
            }
            break;
        //
        // State 3.
        //
        case 3:
            //
            // Continue turning left, if necessary.
            //
            if (drive_using_encoders (-1.0f, 1.0f, -2880, 2880))
            {
                v_state++;
            }
            break;
        //
        // State 4.
        //
        case 4:
            //
            // As soon as the drive encoders reset, begin the next state.
            //
            if (have_drive_encoders_reset ())
            {
                //
                // Begin the next state.
                //
                // There is no conditional (if statement) here, because the
                // motor power won't be applied until this method exits.
                //
                run_without_drive_encoders ();

                //
                // Transition to the next state.
                //
                v_state++;
            }
            break;
        //
        // State 5.
        //
        case 5:
            //
            // Drive forward until a white line is detected.
            //
            //
            // If a white line has been detected, then set the power level to
            // zero.
            //
            if (a_ods_white_tape_detected ())
            {
                set_drive_power (0.0, 0.0);

                //
                // Begin the next state.
                //
                drive_to_ir_beacon ();

                //
                // Transition.
                //
                v_state++;
            }
            //
            // Else a white line has not been detected, so set the power level
            // to full forward.
            //
            else
            {
                set_drive_power (1.0, 1.0);
            }
            break;
        //
        // State 6.
        //
        case 6:
            //
            // When the robot is close to the IR beacon, open the hand.
            //
            int l_status = drive_to_ir_beacon ();
            if (l_status == drive_to_ir_beacon_done)
            {
                //
                // Begin the next state.  Open the claw.
                //
                open_hand ();

                //
                // Transition.
                //
                v_state++;
            }
            else if (l_status == drive_to_ir_beacon_not_detected)
            {
                set_error_message ("IR beacon not detected!");
            }
            break;
        //
        // Perform no action - stay in this case until the OpMode is stopped.
        // This method will still be called regardless of the state machine.
        //
        default:
            //
            // The autonomous actions have been accomplished (i.e. the state has
            // transitioned into its final state.
            //
            break;
        }

        //
        // Update the arm state machine.
        //
        update_arm_state ();

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry
        telemetry.addData ("11", "Drive State: " + v_state);
        telemetry.addData ("12", "Arm State: " + v_arm_state);

    } // loop

    //--------------------------------------------------------------------------
    //
    // update_arm_state
    //
    /**
     * Implement a state machine that controls the arm during auto-operation.
     */
    public void update_arm_state ()

    {
        //
        // Update the arm state machine.
        //
        switch (v_arm_state)
        {
            //
            // State 0.
            //
            case 0:
                //
                // Wait until a command is given (i.e. v_state is set to 1).
                //
                break;
            //
            // State 1.
            //
            case 1:
                //
                // Continue moving the arm up.  If the touch sensor is
                // triggered, then the arm will stop and this call will perform
                // no action.  If the touch sensor has not been triggered, then
                // motor power will still be applied.
                //
                if (move_arm_upward_until_touch ())
                {
                    //
                    // Transition to the stop state.
                    //
                    v_arm_state++;
                }
                break;
            //
            // Perform no action - stay in this case until the OpMode is
            // stopped.  This method will still be called regardless of the
            // state machine.
            //
            default:
                //
                // The autonomous actions have been accomplished (i.e. the state
                // has transitioned into its final state.
                //
                break;
        }

    } // update_arm_state

    //--------------------------------------------------------------------------
    //
    // v_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialize (0).  During the
     * first iteration of the loop method, the state will change from initialize
     * to state_1.  When state_1 actions are complete, the state will change to
     * state_2.  This implements a state machine for the loop method.
     */
    private int v_state = 0;

    //--------------------------------------------------------------------------
    //
    // v_arm_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialize (0).  During the
     * first iteration of the loop method, the state will change from initialize
     * to state_1.  When state_1 actions are complete, the state will change to
     * state_2.  This implements a state machine for the loop method.
     */
    private int v_arm_state = 0;

} // PushBotAutoSensors
