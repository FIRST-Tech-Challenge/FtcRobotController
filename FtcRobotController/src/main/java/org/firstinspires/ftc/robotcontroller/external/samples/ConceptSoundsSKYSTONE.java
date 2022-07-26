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

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This file demonstrates how to play one of the several SKYSTONE/Star Wars sounds loaded into the SDK.
 * It does this by creating a simple "chooser" controlled by the gamepad Up Down buttons.
 * This code also prevents sounds from stacking up by setting a "playing" flag, which is cleared when the sound finishes playing.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * Operation:
 *      Use the DPAD to change the selected sound, and the Right Bumper to play it.
 */

@TeleOp(name="SKYSTONE Sounds", group="Concept")
@Disabled
public class ConceptSoundsSKYSTONE extends LinearOpMode {

    // List of available sound resources
    String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };
    boolean soundPlaying = false;

    @Override
    public void runOpMode() {

        // Variables for choosing from the available sounds
        int     soundIndex      = 0;
        int     soundID         = -1;
        boolean was_dpad_up     = false;
        boolean was_dpad_down   = false;

        Context myApp = hardwareMap.appContext;

        // create a sound parameter that holds the desired player parameters.
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;

        // In this sample, we will skip waiting for the user to press play, and start displaying sound choices right away
        while (!isStopRequested()) {

            // Look for DPAD presses to change the selection
            if (gamepad1.dpad_down && !was_dpad_down) {
                // Go to next sound (with list wrap) and display it
                soundIndex = (soundIndex + 1) % sounds.length;
            }

            if (gamepad1.dpad_up && !was_dpad_up) {
                // Go to previous sound (with list wrap) and display it
                soundIndex = (soundIndex + sounds.length - 1) % sounds.length;
            }

            // Look for trigger to see if we should play sound
            // Only start a new sound if we are currently not playing one.
            if (gamepad1.right_bumper && !soundPlaying) {

                // Determine Resource IDs for the sounds you want to play, and make sure it's valid.
                if ((soundID = myApp.getResources().getIdentifier(sounds[soundIndex], "raw", myApp.getPackageName())) != 0){

                    // Signal that the sound is now playing.
                    soundPlaying = true;

                    // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.
                    SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                            new Runnable() {
                                public void run() {
                                    soundPlaying = false;
                                }} );
                }
            }

            // Remember the last state of the dpad to detect changes.
            was_dpad_up     = gamepad1.dpad_up;
            was_dpad_down   = gamepad1.dpad_down;

            // Display the current sound choice, and the playing status.
            telemetry.addData("", "Use DPAD up/down to choose sound.");
            telemetry.addData("", "Press Right Bumper to play sound.");
            telemetry.addData("", "");
            telemetry.addData("Sound >", sounds[soundIndex]);
            telemetry.addData("Status >", soundPlaying ? "Playing" : "Stopped");
            telemetry.update();
        }
    }
}
