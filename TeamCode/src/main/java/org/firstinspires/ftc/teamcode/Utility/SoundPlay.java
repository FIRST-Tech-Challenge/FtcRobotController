package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.File;

public class SoundPlay extends LinearOpMode {

    // Point to sound files on the phone's drive
    private String soundPath = "/FIRST/blocks/sounds";
    private File sound = new File("/sdcard/FIRST/blocks/sounds/silver.wav");

    // Declare OpMode members.
    private boolean isX = false;    // Gamepad button state variables
    private boolean isB = false;

    private boolean wasX = false;   // Gamepad button history variables
    private boolean WasB = false;

    @Override
    public void runOpMode() {
        boolean silverFound = sound.exists();

        // Display sound status
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
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, sound);
                telemetry.addData("Playing", "Silver File");
                telemetry.update();
            }

            // Save last button states
            wasX = isX;
            WasB = isB;
        }
    }
}
