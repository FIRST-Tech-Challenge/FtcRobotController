package org.firstinspires.ftc.masters;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file demonstrates how to play simple sounds on both the RC and DS phones.
 * It illustrates how to build sounds into your application as a resource.
 * This technique is best suited for use with Android Studio since it assumes you will be creating a new application
 *
 * If you are using OnBotJava, please see the ConceptSoundsOnBotJava sample
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * Operation:
 *
 * Gamepad X & B buttons are used to trigger sounds in this example, but any event can be used.
 * Note: Time should be allowed for sounds to complete before playing other sounds.
 *
 * For sound files to be used as a compiled-in resource, they need to be located in a folder called "raw" under your "res" (resources) folder.
 * You can create your own "raw" folder from scratch, or you can copy the one from the FtcRobotController module.
 *
 *     Android Studio coders will ultimately need a folder in your path as follows:
 *       <project root>/blackswan/src/main/res/raw
 *
 *     Copy any .wav files you want to play into this folder.
 *     Make sure that your files ONLY use lower-case characters, and have no spaces or special characters other than underscore.
 *
 *     The name you give your .wav files will become the resource ID for these sounds.
 *     eg:  gold.wav becomes R.raw.gold
 *
 *     If you wish to use the sounds provided for this sample, they are located in:
 *     <project root>/FtcRobotController/src/main/res/raw
 *     You can copy and paste the entire 'raw' folder using Android Studio.
 *
 */

@TeleOp(name="Concept: Sound Resources", group="Concept")

public class sudio extends LinearOpMode {

    // Declare OpMode members.
    // Sound file present flags
    private boolean sheepFound;

    private boolean isX = false;    // Gamepad button state variables

    private boolean wasX = false;   // Gamepad button history variables

    @Override
    public void runOpMode() {

        // Determine Resource IDs for sounds built into the RC application.
        int sheepSoundID = hardwareMap.appContext.getResources().getIdentifier("sheep", "raw", hardwareMap.appContext.getPackageName());

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (sheepSoundID != 0)
            sheepFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, sheepSoundID);

        // Display sound status
        telemetry.addData("sheep resource", sheepFound ? "Found" : "Not found /src/main/res/raw" );

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to continue");
        telemetry.update();
        waitForStart();

        telemetry.addData(">", "Press X to play sounds.");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // say Silver each time gamepad X is pressed (This sound is a resource)
            if (sheepFound && (isX = gamepad1.x) && !wasX) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, sheepSoundID);
                telemetry.addData("Playing", "Sheep");
                telemetry.update();
            }

            // Save last button states
            wasX = isX;
        }
    }
}