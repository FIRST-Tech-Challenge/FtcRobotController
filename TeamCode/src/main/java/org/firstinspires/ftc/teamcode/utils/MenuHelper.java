package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MenuHelper {
    public static void handleControllerInput(LinearOpMode opMode, Gamepad gamepad, boolean active, Runnable handler) {
        if (active && (gamepad.dpad_up || gamepad.dpad_down || gamepad.a)) {
            handler.run();
            opMode.sleep(250); // Debounce
        }
    }

    public static void displayMenuOptions(Telemetry telemetry, String[] options, int currentSelection) {
        for (int i = 0; i < options.length; i++) {
            telemetry.addData(i == currentSelection ? ">" : " ", options[i]);
        }
    }
}