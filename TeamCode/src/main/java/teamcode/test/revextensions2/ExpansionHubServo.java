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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import static teamcode.test.revextensions2.Utils.getLynxFromController;


/**
 * Extends a ServoImplEx to provide access to new features.
 * Note: servo MUST be attached to an Expansion Hub.
 */

public class ExpansionHubServo extends ServoImplEx
{
    private ExpansionHubEx expansionHubEx;

    /*
     * Don't use this constructor in user-code; this object will be
     * hotswapped into the hardwareMap at runtime.
     */
    ExpansionHubServo(Servo servo)
    {
        super(
                (ServoControllerEx)servo.getController(),
                servo.getPortNumber(),
                servo.getDirection(),
                ServoConfigurationType.getStandardServoType());

        expansionHubEx = new ExpansionHubEx(getLynxFromController((LynxController) servo.getController()));
    }

    /***
     * Set the pulse width output of this servo port directly, rather than using
     * the -1 to 1 range from the SDK
     *
     * @param uS the pulse width (in uS) to set this servo port to
     */
    public void setPulseWidthUs(int uS)
    {
        expansionHubEx.setServoPulseWidth(getPortNumber(), uS);
    }
}
