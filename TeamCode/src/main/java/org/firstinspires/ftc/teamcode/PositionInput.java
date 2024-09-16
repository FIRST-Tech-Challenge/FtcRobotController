package org.firstinspires.ftc.teamcode;

import android.os.Environment;

@TeleOp(name = "PositionInput")
public class PositionInput extends OpMode {
    // The directory that the program stores files in.
    private final File storageFile;

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        telemetry.update();

        if (gamepad1.a || gamepad2.a) {
        } else if (gamepad1.b || gamepad2.b) {
        } else if (gamepad1.x || gamepad2.x) {
        } else if (gamepad1.y || gamepad2.y) {
        }
    }
}