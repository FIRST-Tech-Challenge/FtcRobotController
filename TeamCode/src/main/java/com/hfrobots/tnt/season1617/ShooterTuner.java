/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.season1617;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Provide a basic manual operational mode that allows manual adjustment of particle shooter
 * motor power and displays the values, use dpad up/down for top motor, left/right for bottom
 */
@TeleOp(name="Shooter Tuner")
@Disabled
@SuppressWarnings("unused")
public class ShooterTuner extends VelocityVortexHardware

{
    private double topMotorPower = 0.0D;

    private double bottomMotorPower = 0.0D;

    /*
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public ShooterTuner() {
    }


    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        if (driverDpadUp.getRise()) {
            topMotorPower += 0.05;
        } else if (driverDpadDown.getRise()) {
            topMotorPower -= 0.05;
        }

        topMotorPower = Range.clip(topMotorPower, 0, 1);

        if (driverDpadRight.getRise()) {
            bottomMotorPower += 0.05;
        } else if (driverDpadLeft.getRise()) {
            bottomMotorPower -= 0.05;
        }

        topMotorPower = Range.clip(topMotorPower, 0, 1);

        topParticleShooter.setPower(topMotorPower);
        bottomParticleShooter.setPower(bottomMotorPower);

        telemetry.addData("Shooter top/bottom", "%.3f / %.3f", topMotorPower, bottomMotorPower);
    }
}