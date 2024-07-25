/* Copyright (c) 2018 FIRST. All rights reserved.
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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

/*
 * This OpMode demonstrates how to play simple sounds on both the RC and DS phones.
 * It illustrates how to play sound files that have been copied to the RC Phone
 * This technique is best suited for use with OnBotJava since it does not require the app to be modified.
 *
 * Operation:
 *
 * Gamepad X & B buttons are used to trigger sounds in this example, but any event can be used.
 * Note: Time should be allowed for sounds to complete before playing other sounds.
 *
 *  To play a new sound, you will need to copy the .wav files to the phone, and then provide the full path to them as part of your OpMode.
 *  This is done in this sample for the two sound files.  silver.wav and gold.wav
 *
 *  You can put the files in a variety of soundPaths, but we recommend you put them in the /FIRST/blocks/sounds folder.
 *  Your OpModes will have guaranteed access to this folder, and you can transfer files into this folder using the BLOCKS web page.
 *  --  There is a link called "sounds" on the right hand side of the color bar on the BLOCKS page that can be used to send sound files to this folder by default.
 *  Or you can use Windows File Manager, or ADB to transfer the sound files
 *
 *  To get full use of THIS sample, you will need to copy two sound file called silver.wav and gold.wav to /FIRST/blocks/sounds on the RC phone.
 *  They can be located here:
 *      https://github.com/ftctechnh/ftc_app/tree/master/FtcRobotController/src/main/res/raw/gold.wav
 *      https://github.com/ftctechnh/ftc_app/tree/master/FtcRobotController/src/main/res/raw/silver.wav
 */

@TeleOp(name="Concept: Sound Files", group="Concept")
@Disabled
public class ConceptSoundsOnBotJava extends LinearOpMode {

    // Point to sound files on the phone's drive
    private String soundPath = "/FIRST/blocks/sounds";
    private File goldFile   = new File("/sdcard" + soundPath + "/gold.wav");
    private File silverFile = new File("/sdcard" + soundPath + "/silver.wav");

    // Declare OpMode members.
    private boolean isX = false;    // Gamepad button state variables
    private boolean isB = false;

    private boolean wasX = false;   // Gamepad button history variables
    private boolean WasB = false;

    @Override
    public void runOpMode() {

        // Make sure that the sound files exist on the phone
        boolean goldFound   = goldFile.exists();
        boolean silverFound = silverFile.exists();

        // Display sound status
        telemetry.addData("gold sound",   goldFound ?   "Found" : "NOT Found \nCopy gold.wav to " + soundPath  );
        telemetry.addData("silver sound", silverFound ? "Found" : "NOT Found \nCopy silver.wav to " + soundPath );

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to continue");
        telemetry.update();
        waitForStart();

        telemetry.addData(">", "Press X or B to play sounds.");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // say Silver each time gamepad X is pressed (This sound is a resource)
            if (silverFound && (isX = gamepad1.x) && !wasX) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverFile);
                telemetry.addData("Playing", "Silver File");
                telemetry.update();
            }

            // say Gold each time gamepad B is pressed  (This sound is a resource)
            if (goldFound && (isB = gamepad1.b) && !WasB) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldFile);
                telemetry.addData("Playing", "Gold File");
                telemetry.update();
            }

            // Save last button states
            wasX = isX;
            WasB = isB;
        }
    }
}
