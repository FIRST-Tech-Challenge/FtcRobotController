/*
Copyright (c) 2024 Miriam Sinton-Remes

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This OpMode illustrates using edge detection on a gamepad.
 *
 * Simply checking the state of a gamepad button each time could result in triggering an effect
 * multiple times. Edge detection ensures that you only detect one button press, regardless of how
 * long the button is held.
 *
 * There are two main types of edge detection. Rising edge detection will trigger when a button is
 * first pressed. Falling edge detection will trigger when the button is released.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

@Disabled
@TeleOp(name="Concept: Gamepad Edge Detection", group ="Concept")
public class ConceptGamepadEdgeDetection extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Wait for the DS start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Update the telemetry
            telemetryButtonData();

            // Wait 2 seconds before doing another check
            sleep(2000);
        }
    }

    public void telemetryButtonData() {
        // Add the status of the Gamepad 1 Left Bumper
        telemetry.addData("Gamepad 1 Left Bumper Pressed", gamepad1.leftBumperWasPressed());
        telemetry.addData("Gamepad 1 Left Bumper Released", gamepad1.leftBumperWasReleased());
        telemetry.addData("Gamepad 1 Left Bumper Status", gamepad1.left_bumper);

        // Add an empty line to seperate the buttons in telemetry
        telemetry.addLine();

        // Add the status of the Gamepad 1 Right Bumper
        telemetry.addData("Gamepad 1 Right Bumper Pressed", gamepad1.rightBumperWasPressed());
        telemetry.addData("Gamepad 1 Right Bumper Released", gamepad1.rightBumperWasReleased());
        telemetry.addData("Gamepad 1 Right Bumper Status", gamepad1.right_bumper);

        // Add a note that the telemetry is only updated every 2 seconds
        telemetry.addLine("\nTelemetry is updated every 2 seconds.");

        // Update the telemetry on the DS screen
        telemetry.update();
    }
}
