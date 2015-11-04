package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

//------------------------------------------------------------------------------
//
// PushBotHardwareSensors
//
/**
 * Provides a single sensor access point between custom op-modes and the
 * OpMode class for the Push Bot.  It does this by extending the original Push
 * Bot's hardware and telemetry classes.
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 *
 * @author SSI Robotics
 * @version 2015-08-13-20-04
 */
public class PushBotHardwareSensors extends PushBotTelemetry

{
    //--------------------------------------------------------------------------
    //
    // PushBotHardwareSensors
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotHardwareSensors()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotHardwareSensors

    //--------------------------------------------------------------------------
    //
    // init
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void init ()

    {
        //
        // Use a base class method to associate class members to non-sensor
        // hardware ports (i.e. left/right drive wheels, left arm, etc.).
        //
        super.init ();

        //
        // Connect the sensors.
        //
        try
        {
            v_sensor_touch = hardwareMap.touchSensor.get ("sensor_touch");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("sensor_touch");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_sensor_touch = null;
        }

        try
        {
            v_sensor_ir = hardwareMap.irSeekerSensor.get ("sensor_ir");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("sensor_ir");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_sensor_ir = null;
        }

        try
        {
            v_sensor_ods = hardwareMap.opticalDistanceSensor.get ("sensor_ods");
        }
        catch (Exception p_exeception)
        {
            try
            {
                v_sensor_ods = hardwareMap.opticalDistanceSensor.get
                    ( "sensor_eopd"
                    );
            }
            catch (Exception p_exeception_eopd)
            {
                try
                {
                    v_sensor_ods = hardwareMap.opticalDistanceSensor.get
                        ( "sensor_EOPD"
                        );
                }
                catch (Exception p_exeception_EOPD)
                {
                    m_warning_message ("sensor_ods/eopd/EOPD");
                    DbgLog.msg
                        ( "Can't map sensor_ods nor sensor_eopd, nor" +
                            " sensor_EOPD ("
                        + p_exeception_EOPD.getLocalizedMessage ()
                        + ").\n"
                        );

                    v_sensor_ods = null;
                }
            }
        }

    } // init

    //--------------------------------------------------------------------------
    //
    // is_touch_sensor_pressed
    //
    /**
     * Indicate whether the touch sensor has been pressed.
     */
    boolean is_touch_sensor_pressed ()

    {
        boolean l_return = false;

        if (v_sensor_touch != null)
        {
            l_return = v_sensor_touch.isPressed ();
        }

        return l_return;

    } // is_touch_sensor_pressed

    //--------------------------------------------------------------------------
    //
    // move_arm_upward_until_touch
    //
    /**
     * Apply upward power to the arm motor until the touch sensor is pressed.
     */
    boolean move_arm_upward_until_touch ()

    {
        //
        // If the touch sensor is pressed, halt the motors.
        //
        if (is_touch_sensor_pressed ())
        {
            m_left_arm_power (0.0f);
        }
        //
        // Move the arm upward at full power.
        //
        else
        {
            m_left_arm_power (1.0f);
        }

        //
        // Return whether the sensor has been pressed.
        //
        return is_touch_sensor_pressed ();

    } // move_arm_upward_until_touch

    //--------------------------------------------------------------------------
    //
    // a_ir_angle
    //
    /**
     * Access the IR sensor's estimated angle.
     */
    double a_ir_angle ()

    {
        double l_return = 0.0;

        if (v_sensor_ir != null)
        {
            l_return = v_sensor_ir.getAngle ();
        }

        return l_return;

    } // a_ir_angle

    //--------------------------------------------------------------------------
    //
    // a_ir_strength
    //
    /**
     * Access the IR sensor's estimated strength level.
     */
    double a_ir_strength ()

    {
        double l_return = 0.0;

        if (v_sensor_ir != null)
        {
            l_return = v_sensor_ir.getStrength ();
        }

        return l_return;

    } // a_ir_strength

    //--------------------------------------------------------------------------
    //
    // a_ir_angles_and_strengths
    //
    /**
     * Access the IR sensor's angle and strength levels.
     */
    IrSeekerSensor.IrSeekerIndividualSensor[] a_ir_angles_and_strengths ()

    {
        IrSeekerSensor.IrSeekerIndividualSensor[] l_return =
            { new IrSeekerSensor.IrSeekerIndividualSensor ()
            , new IrSeekerSensor.IrSeekerIndividualSensor ()
            };

        if (v_sensor_ir != null)
        {
            l_return = v_sensor_ir.getIndividualSensors ();
        }

        return l_return;

    } // a_ir_angles_and_strengths

    //--------------------------------------------------------------------------
    //
    // drive_to_ir_beacon
    //
    /**
     * Drive toward the IR beacon.
     */
    final int drive_to_ir_beacon_not_detected = -2;
    final int drive_to_ir_beacon_invalid = -1; // Unknown failure
    final int drive_to_ir_beacon_running = 0;
    final int drive_to_ir_beacon_done = 1;
    int drive_to_ir_beacon ()

    {
        //
        // Assume that the IR beacon is far away.
        //
        int l_return = drive_to_ir_beacon_invalid;

        //
        // Is the IR sensor too far from the beacon?
        //
        double l_strength = a_ir_strength ();
        if ((l_strength > 0.01) && (l_strength < 0.2))
        {
            //
            // Use the estimated angle to determine the direction and power that
            // needs to be applied to the drive motors.
            //
            double l_angle = a_ir_angle () / 240;
            double l_left = Range.clip (0.15 + l_angle, -1, 1);
            double l_right = Range.clip (0.15 - l_angle, -1, 1);

            set_drive_power (l_left, l_right);

            l_return = drive_to_ir_beacon_running;
        }
        //
        // The beacon can't be detected.
        //
        else if (l_strength <= 0.0)
        {
            set_drive_power (0.0, 0.0);

            l_return = drive_to_ir_beacon_not_detected;
        }
        //
        // The beacon is near.
        //
        else
        {
            set_drive_power (0.0, 0.0);

            l_return = drive_to_ir_beacon_done;
        }

        //
        // Return the status.
        //
        return l_return;

    } // drive_to_ir_beacon

    //--------------------------------------------------------------------------
    //
    // a_ods_light_detected
    //
    /**
     * Access the amount of light detected by the Optical Distance Sensor.
     */
    double a_ods_light_detected ()

    {
        double l_return = 0.0;

        if (v_sensor_ods != null)
        {
            v_sensor_ods.getLightDetected ();
        }

        return l_return;

    } // a_ods_light_detected

    //--------------------------------------------------------------------------
    //
    // a_ods_white_tape_detected
    //
    /**
     * Access whether the EOP is detecting white tape.
     */
    boolean a_ods_white_tape_detected ()

    {
        //
        // Assume not.
        //
        boolean l_return = false;

        if (v_sensor_ods != null)
        {
            //
            // Is the amount of light detected above the threshold for white
            // tape?
            //
            if (v_sensor_ods.getLightDetected () > 0.8)
            {
                l_return = true;
            }
        }

        //
        // Return
        //
        return l_return;

    } // a_ods_white_tape_detected

    //--------------------------------------------------------------------------
    //
    // v_sensor_touch
    //
    /**
     * Manage the touch sensor.
     */
    private TouchSensor v_sensor_touch;

    //--------------------------------------------------------------------------
    //
    // v_sensor_ir
    //
    /**
     * Manage the infra-red sensor.
     */
    private IrSeekerSensor v_sensor_ir;

    //--------------------------------------------------------------------------
    //
    // v_sensor_ods
    //
    /**
     * Manage the optical distance sensor.
     */
    private OpticalDistanceSensor v_sensor_ods;

} // PushBotHardwareSensors
