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

package org.firstinspires.ftc.teamcode.Sensors;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "sensor_ods".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensors: Touch And Distance", group = "Sensor")
public class SensorMROpticalDistance extends LinearOpMode {

  DistanceSensor odsSensor;  // Hardware Device Object
  TouchSensor digitalTouch;  // Hardware Device Object

  // Define class members
  SlowServo servo1;
  SlowServo servo2;

  @Override
  public void runOpMode() {

    // get a reference to our Light Sensor object.
    odsSensor = hardwareMap.get(DistanceSensor.class, GreenhillHardware.DISTANCE_SENSOR);

    // get a reference to our digitalTouch object.
    digitalTouch = hardwareMap.get(TouchSensor.class, GreenhillHardware.TOUCH_SENSOR);
    servo1 = new SlowServo(hardwareMap.servo.get(GreenhillHardware.RIGHT_SERVO));
    servo2 = new SlowServo(hardwareMap.servo.get(GreenhillHardware.LEFT_SERVO));

    // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the light levels.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      // send the info back to driver station using telemetry function.
      // if the digital channel returns true it's HIGH and the button is unpressed.
      if (digitalTouch.isPressed()) {
        servo1.setTarget(1.0);
        telemetry.addData("Digital Touch", "Is Pressed");
      } else {
        servo1.setTarget(0.0);
        telemetry.addData("Digital Touch", "Is Not Pressed");
      }
      double dist = odsSensor.getDistance(DistanceUnit.CM);
      if (dist > 50){
        servo2.setTarget(1.0);
      }else{
        servo2.setTarget(0.0);
      }
      // send the info back to driver station using telemetry function.
      telemetry.addData("Distance",    dist);
      servo1.update();
      servo2.update();
      telemetry.update();
      idle();

    }
  }
}
