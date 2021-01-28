/*
 * Copyright (c) 2017 OpenFTC Team
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

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import static teamcode.test.revextensions2.Utils.getLynxFromController;


/**
 * Extends a DcMotorImplEx to provide access to new features.
 * Note: motor MUST be attached to an Expansion Hub.
 */
public class ExpansionHubMotor extends DcMotorImplEx
{
    private ExpansionHubEx expansionHubEx;

    /*
     * Don't use this constructor in user-code; this object will be
     * hotswapped into the hardwareMap at runtime.
     */
    ExpansionHubMotor(DcMotor motor)
    {
        super(motor.getController(), motor.getPortNumber(), motor.getDirection(), motor.getMotorType());
        expansionHubEx = new ExpansionHubEx(getLynxFromController((LynxController) motor.getController()));
    }

    /**
     * Get the amount of current this motor is pulling from its H-bridge
     *
     * @return the current draw in milliamps
     */
    public double getCurrentDraw(ExpansionHubEx.CurrentDrawUnits units)
    {
        return expansionHubEx.getMotorCurrentDraw(units, getPortNumber());
    }

    /***
     * Query as to whether the H-bridge for this motor is over-temp
     *
     * @return boolean indicating the H-bridge is over-temp
     */
    public boolean isBridgeOverTemp()
    {
        return expansionHubEx.isMotorBridgeOverTemp(getPortNumber());
    }

    /***
     * Query as to whether this motor has lost (encoder?) counts
     * @deprecated because I have no idea what this actually does
     *
     * @return boolean indicating whether the motor has lost (encoder?) counts
     */
    @Deprecated
    public boolean hasLostCounts()
    {
        return expansionHubEx.hasMotorLostCounts(getPortNumber());
    }
}