/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/*
Sample written by Coach for a very simple reading of analog inputs
Must configure individual analog inputs
*/
@TeleOp(name = "Sensor: Analog In", group = "Sensor")
//@Disabled
public class SensorAnalogIn extends LinearOpMode {

final int BLUE_LED_CHANNEL = 0;
final int RED_LED_CHANNEL = 1;

  @Override
  public void runOpMode() {

    AnalogInput        stoneLeft;                // Device Object
    AnalogInput        stoneRight;               // Device Object
Servo servo1;

    stoneLeft  = hardwareMap.get(AnalogInput.class, "leftAnalog");     //  Use generic form of device mapping
    stoneRight = hardwareMap.get(AnalogInput.class, "rightAnalog");    //  Use generic form of device mapping
    servo1 = hardwareMap.get(Servo.class, "servo1");

    // wait for the start button to be pressed.
    telemetry.addData(">", "Press play to start readings");
    telemetry.update();
    waitForStart();

    while (opModeIsActive())  {

        if (stoneLeft.getVoltage() > 2)  servo1.setPosition(0.5);
        else servo1.setPosition(0.0);

        telemetry.addData("Left", stoneLeft.getVoltage() );
        telemetry.addData("Right", stoneRight.getVoltage() );

        telemetry.update();
    }
  }
}
