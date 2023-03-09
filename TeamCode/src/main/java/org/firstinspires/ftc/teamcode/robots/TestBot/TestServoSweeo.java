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

package org.firstinspires.ftc.teamcode.robots.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Taubot: Test Servo Sweep", group = "Test")
//@Disabled
public class TestServoSweeo extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   100;     // period of each cycle
    static final double MAX_POS     =  0.6;     // Maximum rotational position
    static final double MIN_POS     =  0.4;     // Minimum rotational position

    // Define class members
    Servo servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    DcMotorEx motor0, motor1, motor2, motor3;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo1 = hardwareMap.get(Servo.class, "servoGripper");
        servo2 = hardwareMap.get(Servo.class, "nudgeSwivel");
        servo3 = hardwareMap.get(Servo.class, "shoulderJoint");
        servo4 = hardwareMap.get(Servo.class, "elbowJoint");
        servo5 = hardwareMap.get(Servo.class, "wristServo");
        servo6 = hardwareMap.get(Servo.class, "lassoJoint");
        servo7 = hardwareMap.get(Servo.class, "turretServo");
        servo8 = hardwareMap.get(Servo.class, "testServo");

        motor0 = hardwareMap.get(DcMotorEx.class, "motorRight");
        motor1 = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motor2 = hardwareMap.get(DcMotorEx.class, "motorChariot");
        motor3 = hardwareMap.get(DcMotorEx.class, "test");

        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        // Wait for the start button
        telemetry.addData(">", "Press Start to wiggle Servos." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
/*
            motor0.setPower(0.2);
            motor1.setPower(0.2);
            motor2.setPower(0.2);
            motor3.setPower(0.2);
*/
            telemetry.addData("Motor0", motor0.getCurrentPosition());
            telemetry.addData("Motor1", motor1.getCurrentPosition());
            telemetry.addData("Motor2", motor2.getCurrentPosition());
            telemetry.addData("Motor3", motor3.getCurrentPosition());

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            servo1.setPosition(position);
            servo2.setPosition(position);
            servo3.setPosition(position);
            servo4.setPosition(position);
            servo5.setPosition(position);
            servo6.setPosition(position);
            servo7.setPosition(position);
            servo8.setPosition(position);

            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
