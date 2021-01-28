/*
 * Copyright (c) 2018 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.test.revextensions2;

import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxDigitalChannelController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;


import java.lang.reflect.Field;

/**
 * This class is a wrapper over LynxGetBulkInputDataResponse that provides
 * easier-to-use methods to access the underlying data
 */

public class RevBulkData
{
    private LynxGetBulkInputDataResponse response;
    private LynxModule module;

    /***
     * Constructor
     *
     * @param response the raw response over which we are to be a wrapper
     *
     * @param module the Lynx module that this response came from;
     *               used to verify that this response actually
     *               contains the device information the user
     *               was after. (They might have issued this command
     *               to the wrong LynxModule)
     */
    RevBulkData(LynxGetBulkInputDataResponse response, LynxModule module)
    {
        this.response = response;
        this.module = module;
    }

    // see DcMotorImpl#getOperationalDirection()
    private static DcMotorSimple.Direction getMotorOperationalDirection(DcMotor dcMotor)
    {
        MotorConfigurationType motorType = dcMotor.getMotorType();
        DcMotorSimple.Direction direction = dcMotor.getDirection();
        if (motorType.getOrientation() == Rotation.CCW)
        {
            return direction.inverted();
        }
        else
        {
            return direction;
        }
    }

    //-----------------------------------------------------------------------------------
    // Encoder counts
    //-----------------------------------------------------------------------------------

    /***
     * Get the encoder count of a motor port on the Expansion Hub
     * from which this bulk data response came. Please note that
     * the sign of the value will NOT be adjusted for whether or
     * not the motor was set to reverse mode; for that please use
     * the corresponding method that takes a DcMotor parameter
     * instead of the raw port number.
     *
     * @param motorNum the motor port for which to get the
     *                 encoder count [0-3]
     *
     * @return the encoder count (retrieved from this bulk packet)
     * of the motor port specified by the motorNum parameter
     */
    public int getMotorCurrentPosition(int motorNum)
    {
        return response.getEncoder(motorNum);
    }

    /***
     * Gets the encoder count for a motor that is connected to the
     * Expansion Hub from which this bulk data packet came. If the
     * user passes in a motor that is connected to a different Hub
     * than the one that this packet came from, an exception is thrown
     * Additionally, the sign of the value is adjusted to account for
     * whether the user set the motor direction to reverse.
     *
     * @param motor the motor for which to retrieve the encoder count
     *
     * @return the encoder count (retrieved from this bulk packet) for
     *         the DcMotor object passed in
     */
    public int getMotorCurrentPosition(DcMotor motor)
    {
        throwIfMotorInvalid(motor);
        int position = getMotorCurrentPosition(motor.getPortNumber());
        if (getMotorOperationalDirection(motor) == DcMotorSimple.Direction.REVERSE)
        {
            return -position;
        }
        else
        {
            return position;
        }
    }

    //-----------------------------------------------------------------------------------
    // Encoder velocities
    //-----------------------------------------------------------------------------------

    /***
     * Get the encoder velocity of a motor port on the Expansion Hub
     * from which this bulk data response came. Please note that
     * the sign of the value will NOT be adjusted for whether or
     * not the motor was set to reverse mode; for that please use
     * the corresponding method that takes a DcMotor parameter
     * instead of the raw port number.
     *
     * @param motorNum the motor port for which to get the
     *                 encoder velocity [0-3]
     *
     * @return the encoder velocity (retrieved from this bulk packet)
     *         of the motor port specified by the motorNum parameter
     */
    public int getMotorVelocity(int motorNum)
    {
        return response.getVelocity(motorNum);
    }

    /***
     * Gets the encoder velocity for a motor that is connected to the
     * Expansion Hub from which this bulk data packet came. If the
     * user passes in a motor that is connected to a different Hub
     * than the one that this packet came from, an exception is thrown
     * Additionally, the sign of the value is adjusted to account for
     * whether the user set the motor direction to reverse.
     *
     * @param motor the motor for which to retrieve the encoder velocity
     *
     * @return the encoder velocity (retrieved from this bulk packet) for
     *         the DcMotor object passed in
     */
    public int getMotorVelocity(DcMotor motor)
    {
        throwIfMotorInvalid(motor);
        int velocity = getMotorVelocity(motor.getPortNumber());
        if (getMotorOperationalDirection(motor) == DcMotorSimple.Direction.REVERSE)
        {
            return -velocity;
        }
        else
        {
            return velocity;
        }
    }

    //-----------------------------------------------------------------------------------
    // Position control at target position
    //-----------------------------------------------------------------------------------

    /***
     * Checks whether a motor on a given port of the Expansion Hub
     * from which this bulk data response came is at it's target
     * encoder position
     *
     * @param motorNum the port for which to check whether the motor
     *                 is at it's target encoder position
     *
     * @return whether the motor in the specified port is at its target
     *         encoder position as retrieved from this bulk packet
     */
    public boolean isMotorAtTargetPosition(int motorNum)
    {
        return response.isAtTarget(motorNum);
    }

    /***
     * Checks whether a given motor connected to the Expansion Hub
     * from which this bulk data response came is at it's target
     * encoder position. If the user passes in a motor that is connected
     * to a different Hub than the one that this packet came from, an
     * exception is thrown
     *
     * @param motor the motor for which to check if the target encoder
     *              position has been reached
     *
     * @return whether the motor in the specified port is at its target
     *         encoder position as retrieved from this bulk packet
     */
    public boolean isMotorAtTargetPosition(DcMotor motor)
    {
        throwIfMotorInvalid(motor);
        return isMotorAtTargetPosition(motor.getPortNumber());
    }

    //-----------------------------------------------------------------------------------
    // Analog I/Os
    //-----------------------------------------------------------------------------------

    /***
     * Gets the ADC value of an analog pin on the Expansion Hub from
     * which this bulk data packet came.
     *
     * @param pin the analog pin for which to get the ADC value [0-3]
     *
     * @return the ADC value (retrieved from this bulk packet) for the
     *         specified analog pin
     */
    public int getAnalogInputValue(int pin)
    {
        return response.getAnalogInput(pin);
    }

    /***
     * Gets the ADC value of an analog device that is connected to the
     * Expansion Hub from which this bulk data packet came. If the
     * user passes in a device that is connected to a different Hub
     * than the one that this packet came from, an exception is thrown
     *
     * @param input the analog device for which to get the ADC value
     *
     * @return the ADC value (retrieved from this bulk packet) for the
     *         AnalogInput object passed in
     */
    public int getAnalogInputValue(AnalogInput input)
    {
        AnalogInputController controller = null;
        int port = -1;

        try
        {
            Field controllerField = AnalogInput.class.getDeclaredField("controller");
            controllerField.setAccessible(true);
            controller = (AnalogInputController) controllerField.get(input);

            Field channelField = AnalogInput.class.getDeclaredField("channel");
            channelField.setAccessible(true);
            port = (int) channelField.get(input);
        }
        catch (Exception e)
        {
            throw new RE2Exception("Failed to reflect on AnalogInput! Please report this as an issue on the GitHub repository.");
        }

        if(!(controller instanceof LynxAnalogInputController))
        {
            throw new RevBulkDataException(String.format("AnalogInput %s is not attached to a Lynx module!", Utils.getHwMapName(input)));
        }

        if(!validateLynxController((LynxController) controller))
        {
            throw new RevBulkDataException(String.format("AnalogInput %s is attached to a different Lynx module than the one that this bulk command was issued to!", Utils.getHwMapName(input)));
        }

        return getAnalogInputValue(port);
    }

    //-----------------------------------------------------------------------------------
    // Digital I/Os
    //-----------------------------------------------------------------------------------

    /***
     * Gets the state of a digital pin on the Expansion Hub from which
     * this bulk data packet came.
     *
     * @param pin the digital pin for which to get the state [0-7]
     *
     * @return the state (retrieved from this bulk packet) of the
     *         specified digital pin
     */
    public boolean getDigitalInputState(int pin)
    {
        return response.getDigitalInput(pin);
    }

    /***
     * Gets the state of a digital device that is connected to the
     * Expansion Hub from which this bulk data packet came. If the
     * user passes in a device that is connected to a different Hub
     * than the one that this packet came from, an exception is thrown
     *
     * @param digitalChannel the digital device for which to get the state
     *
     * @return the state (retrieved from this bulk packet) for the
     *         DigitalChannel object passed in
     */
    public boolean getDigitalInputState(DigitalChannel digitalChannel)
    {
        DigitalChannelController controller = null;
        int port = -1;

        try
        {
            Field controllerField = DigitalChannelImpl.class.getDeclaredField("controller");
            controllerField.setAccessible(true);
            controller = (DigitalChannelController) controllerField.get(digitalChannel);

            Field channelField = DigitalChannelImpl.class.getDeclaredField("channel");
            channelField.setAccessible(true);
            port = (int) channelField.get(digitalChannel);
        }
        catch (Exception e)
        {
            throw new RE2Exception("Failed to reflect on DigitalChannelImpl! Please report this as an issue on the GitHub repository.");
        }

        if(!(controller instanceof LynxDigitalChannelController))
        {
            throw new RevBulkDataException(String.format("DigitalChannel %s is not attached to a Lynx module!", Utils.getHwMapName(digitalChannel)));
        }

        if(!validateLynxController((LynxController) controller))
        {
            throw new RevBulkDataException(String.format("DigitalChannel %s is attached to a different Lynx module than the one that this bulk command was issued to!", Utils.getHwMapName(digitalChannel)));
        }

        return getDigitalInputState(port);
    }

    //-----------------------------------------------------------------------------------
    // Utility functions
    //-----------------------------------------------------------------------------------

    /***
     * Checks to see if the motor passed in is not attached to a Lynx module,
     * or if it is attached to a different Lynx module than the one that this
     * bulk packet came from. If either of those two conditions is true, an
     * exception is thrown.
     *
     * @param motor the motor to validate
     */
    private void throwIfMotorInvalid(DcMotor motor)
    {
        // Is this motor attached to a Lynx module?
        if(!(motor.getController() instanceof LynxDcMotorController))
        {
            throw new RevBulkDataException(String.format("Motor %s is not attached to a Lynx module!", Utils.getHwMapName(motor)));
        }

        // Is this motor attached to a different Lynx module than the one
        // that this packet came from?
        if(!validateLynxController((LynxController) motor.getController()))
        {
            throw new RevBulkDataException(String.format("Motor %s is attached to a different Lynx module than the one that this bulk command was issued to!", Utils.getHwMapName(motor)));
        }
    }

    /***
     * Checks to see whether a Lynx controller is attached to the Lynx
     * module that this packet came from
     *
     * @param controller the controller to validate
     *
     * @return whether or not the controller is attached to the Lynx
     *         module that this packet came from
     */
    private boolean validateLynxController(LynxController controller)
    {
        return validateLynxModule(Utils.getLynxFromController(controller));
    }

    /***
     * Checks to see whether a Lynx module passed in is the same one
     * that this packet came from
     *
     * @param moduleToValidate the module to validate
     *
     * @return whether or not a Lynx module passed in is the same one
     * that this packet came from
     */
    private boolean validateLynxModule(LynxModule moduleToValidate)
    {
        //TODO: can we just replace with an '=='?
        return moduleToValidate.getModuleAddress() == module.getModuleAddress();
    }
}
