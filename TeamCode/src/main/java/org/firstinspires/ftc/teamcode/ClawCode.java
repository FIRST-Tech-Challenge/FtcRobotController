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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo left_position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo left_position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Concept: Claw_Prototype", group = "Concept")
//@Disabled
public class ClawCode extends LinearOpMode {

    static final double INCREMENT   = 0.125;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS_LEFT =  1.0;     // Maximum rotational left_position
    static final double MIN_POS_LEFT =  0.75;
    static final double MIN_POS_RIGHT =  0.25;
    static final double MAX_POS_RIGHT =  0.0;   // Minimum rotational left_position


    // Define class members
    Servo left_servo;
    Servo right_servo;
    double  left_position = 1.0; // Start at halfway left_position
    double  right_position = 0; // Start at halfway right_position
    //    double  left_position = (MAX_POS_LEFT - MAX_POS_RIGHT) / 2; // Start at halfway left_position
//    double  right_position = (MAX_POS_LEFT - MAX_POS_RIGHT) / 2; // Start at halfway right_position
    //boolean rampUp = true;
    //boolean rampUp = this.gamepad1.left_trigger;
    float ltrigger;
    boolean lbumper;
    float close = 0;
    boolean release = false;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        left_servo = hardwareMap.get(Servo.class, "left_hand");
        right_servo = hardwareMap.get(Servo.class, "right_hand");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        left_position = left_servo.getPosition();
        right_position = right_servo.getPosition();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            ltrigger = this.gamepad1.left_trigger;
            lbumper = this.gamepad1.left_bumper;

            if (ltrigger > 0) {
                close = 1;
            }
            else {
                close = 0;
            }

            if (lbumper == true) {
                release = true;
            }
            else {
                release = false;
            }

            // slew the servo, according to the rampUp (direction) variable.
            if ((close > 0) && (close <= 2)){
                // Keep stepping up until we hit the max value.
                left_position += INCREMENT ;
                if (left_position >= MAX_POS_LEFT) {
                    left_position = MAX_POS_LEFT;
                    close +=1;   // Switch ramp direction
                }
                right_position -= INCREMENT ;
                if (right_position <= MAX_POS_RIGHT) {
                    right_position = MAX_POS_RIGHT;
                    //close = !close;   // Switch ramp direction
                }

            }



            if (release) {
                // Keep stepping down until we hit the min value.
                left_position -= INCREMENT ;
                if (left_position <= MAX_POS_RIGHT) {
                    left_position = MAX_POS_RIGHT;
                    //release = !release;  // Switch ramp direction
                }
                right_position += INCREMENT ;
                if (right_position >= MAX_POS_LEFT) {
                    right_position = MAX_POS_LEFT;
                    //close = !close;   // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", left_position);
            telemetry.addData("Servo Position", "%5.2f", right_position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new left_position and pause;
            if (((close < 2) && (close >0) || (release))) {
                left_servo.setPosition(left_position);
                right_servo.setPosition(right_position);
            }
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}