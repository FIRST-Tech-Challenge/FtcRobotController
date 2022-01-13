package org.firstinspires.ftc.masters;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test Rick Roll")
public class TestRickRoll extends LinearOpMode {

    private boolean rickAstleyFound;

    private boolean isX = false;

    private boolean wasX = false;
    @Override
    public void runOpMode() throws InterruptedException {

        int rickRollID = hardwareMap.appContext.getResources().getIdentifier("rickroll", "raw", hardwareMap.appContext.getPackageName());

        if (rickRollID != 0)
            rickAstleyFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, rickRollID);

        // Display sound status
        telemetry.addData("resource", rickAstleyFound ? "Found" : "Not found\n Add rickroll.wav to /src/main/res/raw" );

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to continue");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            // say Silver each time gamepad X is pressed (This sound is a resource)
            if (rickAstleyFound && (isX = gamepad1.x) && !wasX) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, rickRollID);
                telemetry.addData("Playing", "Resource");
                telemetry.update();
            }
            wasX = isX;

        }
    }
}
