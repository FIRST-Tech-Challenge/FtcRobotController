/**
 Copyright (c) 2017 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1819;

public abstract class RoverRuckusTelemetry extends RoverRuckusHardware {

    protected void updatePriorityTelemetry(String message) {
        telemetry.addData("00", message);
    }

    /**
     * Update the telemetry with current values from the hardware.
     */
    public void updateTelemetry() {
        if (wasWarningGenerated()) {
            setFirstMessage(getWarningMessage());
        }

        //
        // Send telemetry data to the driver station.
        //
        telemetry.addData("pos", "y: " + mecanumDrive.getYPosition() + ", x: " + mecanumDrive.getXPosition());
    }

    public void updateGamepadTelemetry() {
        //
        // Send telemetry data concerning gamepads to the driver station.
        //
        telemetry.addData ("06", "GP1 Left x: " + -gamepad1.left_stick_x);
        telemetry.addData ("07", "GP1 Left y: " + -gamepad1.left_stick_y);
        telemetry.addData ("08", "GP1 Right x: " + -gamepad1.right_stick_x);
        telemetry.addData ("09", "GP1 Right y: " + -gamepad1.right_stick_y);
    }

    public void setFirstMessage(String message) {
        telemetry.addData ( "00", message);
    }


    /**
     * Update the telemetry's first message to indicate an error.
     */
    public void setErrorMessage(String message) {
        setFirstMessage("ERROR: " + message);
    }

}