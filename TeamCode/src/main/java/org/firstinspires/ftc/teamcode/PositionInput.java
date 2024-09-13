package org.firstinspires.ftc.teamcode;

import android.os.Environment;

@TeleOp(name = "PositionInput")
public class PositionInput extends OpMode {
    // The directory that the program stores files in.
    private final File storageFile;

    @Override
    public void init() {
        String storagePath = android.os.Environment.getExternalStorage().getPath() + "/" + fileDir;
    }

    @Override
    public void loop() {
        telemetry.update();
        
        // If "A" is pressed on either gamepad
        if (gamepad1.a || gamepad2.a) {
        }
        
        // If "B" is pressed on either gamepad
        else if (gamepad1.b || gamepad2.b) {

        }

        // If "X" is pressed on either gamepad
        else if (gamepad1.x || gamepad2.x) {

        }
        
        // If "Y" is pressed on either gamepad
        else if (gamepad1.y || gamepad2.y) {
            
        }
    }
}