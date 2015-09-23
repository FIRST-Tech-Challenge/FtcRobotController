/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Example autonomous program.
 * <p>
 * This example program uses elapsed time to determine how to move the robot.
 * The OpMode.java class has some class members that provide time information
 * for the current op mode.
 * The public member variable 'time' is updated before each call to the run() event.
 * The method getRunTime() returns the time that has elapsed since the op mode
 * starting running to when the method was called.
 */
public class K9AutoTime extends OpMode {

	final static double MOTOR_POWER = 0.15; // Higher values will cause the robot to move faster
	final static double HOLD_IR_SIGNAL_STRENGTH = 0.20; // Higher values will cause the robot to follow closer
	final static double LIGHT_THRESHOLD = 0.5;

	double armPosition;
	double clawPosition;

	DcMotor motorRight;
	DcMotor motorLeft;
	Servo claw;
	Servo arm;
	LightSensor reflectedLight;

	/**
	 * Constructor
	 */
	public K9AutoTime() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {

		/*
		 * Use the hardwareMap to get the dc motors and servos by name.
		 * Note that the names of the devices must match the names used
		 * when you configured your robot and created the configuration file.
		 */
		
		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot..
		 *   
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
		motorRight = hardwareMap.dcMotor.get("motor_2");
		motorLeft = hardwareMap.dcMotor.get("motor_1");
		motorLeft.setDirection(DcMotor.Direction.REVERSE);
		
		arm = hardwareMap.servo.get("servo_1");
		claw = hardwareMap.servo.get("servo_6");

		// set the starting position of the wrist and claw
		armPosition = 0.4;
		clawPosition = 0.25;

		/*
		 * We also assume that we have a LEGO light sensor
		 * with a name of "light_sensor" configured for our robot.
		 */
		reflectedLight = hardwareMap.lightSensor.get("light_sensor");

        // turn on LED of light sensor.
        reflectedLight.enableLed(true);
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		double reflection = 0.0;
		double left, right = 0.0;
		
		// keep manipulator out of the way.
		arm.setPosition(armPosition);
		claw.setPosition(clawPosition);


        /*
         * Use the 'time' variable of this op mode to determine
         * how to adjust the motor power.
         */
        if (this.time <= 1) {
            // from 0 to 1 seconds, run the motors for five seconds.
            left = 0.15;
            right = 0.15;
        } else if (this.time > 5 && this.time <= 8.5) {
            // between 5 and 8.5 seconds, point turn right.
            left = 0.15;
            right = -0.15;
        } else if (this.time > 8.5 && this.time <= 15) {
            // between 8 and 15 seconds, idle.
            left = 0.0;
            right = 0.0;
        } else if (this.time > 15d && this.time <= 20.75d) {
            // between 15 and 20.75 seconds, point turn left.
            left = -0.15;
            right = 0.15;
        } else {
            // after 20.75 seconds, stop.
            left = 0.0;
            right = 0.0;
        }

		/*
		 * set the motor power
		 */
        motorRight.setPower(left);
        motorLeft.setPower(right);

		/*
		 * read the light sensor.
		 */
		//reflection = reflectedLight.getLightLevel();
		
		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

		telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
        telemetry.addData("reflection", "reflection:  " + Double.toString(reflection));
		telemetry.addData("left tgt pwr",  "left  pwr: " + Double.toString(left));
		telemetry.addData("right tgt pwr", "right pwr: " + Double.toString(right));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

}
