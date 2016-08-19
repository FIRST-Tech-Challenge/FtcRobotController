/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.internal.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cControllerPortDevice;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * {@link TestMotorControllerFlavors} is a simple test that explores the behavior of
 */
@Autonomous(name = "Test MC Flavors", group = "Tests")
@Disabled
public class TestMotorControllerFlavors extends LinearOpMode
    {
    // We don't require all of these motors to be attached; we'll deal with what we find
    DcMotor legacyMotor;
    DcMotor v15Motor;
    DcMotor v2Motor;

    @Override public void runOpMode() throws InterruptedException
        {
        legacyMotor = findMotor("legacy motor");
        v15Motor    = findMotor("v1.5 motor");
        v2Motor     = findMotor("v2 motor");

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Make the encoders be something other than zero
        reportMotors();
        setPowers(0.5);
        Thread.sleep(3000);
        setPowers(0);
        reportMotors();
        Thread.sleep(3000);
        reportMotors();

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sit there and watch the motors
        while (opModeIsActive())
            {
            reportMotors();
            idle();
            }
        }

    DcMotor findMotor(String motorName)
        {
        try {
            DcMotor motor = hardwareMap.dcMotor.get(motorName);

            // Find which device is the USB device
            RobotArmingStateNotifier usbDevice = null;
            if (motor.getController() instanceof RobotArmingStateNotifier)
                {
                usbDevice = (RobotArmingStateNotifier) motor.getController();
                }
            else if (motor.getController() instanceof I2cControllerPortDevice)
                {
                I2cControllerPortDevice i2cControllerPortDevice = (I2cControllerPortDevice)motor.getController();
                if (i2cControllerPortDevice.getI2cController() instanceof RobotArmingStateNotifier)
                    {
                    usbDevice = (RobotArmingStateNotifier) i2cControllerPortDevice.getI2cController();
                    }
                }

            // Weed out controllers that aren't actually physically there (and so in pretend state)
            if (usbDevice != null)
                {
                if (usbDevice.getArmingState() != RobotArmingStateNotifier.ARMINGSTATE.ARMED)
                    {
                    return null;
                    }
                }

            return motor;
            }
        catch (Throwable ignored)
            {
            }
        return null;
        }

    void setModes(DcMotor.RunMode mode)
        {
        if (legacyMotor != null) legacyMotor.setMode(mode);
        if (v15Motor != null)    v15Motor.setMode(mode);
        if (v2Motor != null)     v2Motor.setMode(mode);
        }

    void setPowers(double power)
        {
        if (legacyMotor != null) legacyMotor.setPower(power);
        if (v15Motor != null)    v15Motor.setPower(power);
        if (v2Motor != null)     v2Motor.setPower(power);
        }

    void reportMotors()
        {
        telemetry.addData("Motor Report", "");
        reportMotor("legacy motor: ", legacyMotor);
        reportMotor("v1.5 motor: ", v15Motor);
        reportMotor("v2 motor: ", v2Motor);
        telemetry.update();
        }

    void reportMotor(String caption, DcMotor motor)
        {
        if (motor != null)
            {
            int position = motor.getCurrentPosition();
            DcMotor.RunMode mode = motor.getMode();

            telemetry.addLine(caption)
                .addData("pos", "%d", position)
                .addData("mode", "%s", mode.toString());

            RobotLog.i("%s pos=%d mode=%s", caption, position, mode.toString());
            }
        }

    }

