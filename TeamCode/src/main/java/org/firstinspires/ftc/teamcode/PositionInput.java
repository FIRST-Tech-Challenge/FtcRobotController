package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import java.nio.file;

import com.apple.eio.FileManager;

@TeleOp(name = "PositionInput")
public class PositionInput extends OpMode {
    // File name of the storage file,
    // which is inside the directory specified in FileManager
    private final Path storageFile = Paths("position.txt");

    @Override
    public void init() {
        telemetry.addData("Current position: ", FileManager.readFile(storageFile));
    }

    @Override
    public void loop() {
        telemetry.update();

        String positionString = null;
        // Green button
        if (gamepad1.a || gamepad2.a) {
            positionString = StartingPosition.BLUE_NEAR.name();

        } else if (gamepad1.b || gamepad2.b) {
            // Red button
            positionString = StartingPosition.RED_FAR.name();

        } else if (gamepad1.x || gamepad2.x) {
            // Blue button
            positionString = StartingPosition.BLUE_FAR.name();

        } else if (gamepad1.y || gamepad2.y) {
            // Orange button
            positionString = StartingPosition.RED_NEAR.name();
        }

        if (positionString != null) {
            FileManager.writeToFile(storageFile, positionString);
            telemetry.addData("Current position: ", FileManager.readFile(storageFile));
        }
    }
}