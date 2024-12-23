package org.firstinspires.ftc.teamcode.tatooine.utils.Alliance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.IOException;

/**
 * A simple OpMode to switch between Red and Blue alliances by creating or deleting a file named
 * "red.txt" on the phone's storage. The static method {@link CheckAlliance#isRed()} determines
 * which alliance is active by checking if the file exists.
 */
@TeleOp(name = "Alliance Selector", group = "Utils")
public class AllianceSelector extends LinearOpMode {

    // Constants for file storage
    private static final String BASE_PATH    = "/sdcard/FIRST/";
    private static final String RED_FILE     = "red.txt";

    // State for alliance selection
    private boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // File object pointing to "/sdcard/FIRST/red.txt"
        File redFile = new File(BASE_PATH + RED_FILE);

        // Determine current alliance from CheckAlliance
        isRed = CheckAlliance.isRed();

        // Give user feedback on current alliance
        telemetry.addData("Current Alliance", isRed ? "Red" : "Blue");
        telemetry.addLine("Press START to toggle alliance.");
        telemetry.update();

        // Wait for the user to press START
        waitForStart();

        if (opModeIsActive()) {
            if (isRed) {
                /*
                 * If we are currently Red, switching to Blue means we remove "red.txt".
                 */
                if (redFile.exists()) {
                    boolean deleted = redFile.delete();
                    if (deleted) {
                        telemetry.addLine("Alliance changed: Now Blue.");
                        telemetry.addLine("'red.txt' has been removed.");
                    } else {
                        telemetry.addLine("Error: Could not delete 'red.txt'.");
                    }
                } else {
                    telemetry.addLine("'red.txt' not found (already Blue?).");
                }
            } else {
                /*
                 * If we are currently Blue, switching to Red means we create "red.txt".
                 */
                try {
                    boolean created = redFile.createNewFile();
                    if (created) {
                        telemetry.addLine("Alliance changed: Now Red.");
                        telemetry.addLine("'red.txt' has been created.");
                    } else {
                        telemetry.addLine("'red.txt' already exists (already Red?).");
                    }
                } catch (IOException e) {
                    telemetry.addLine("Error creating 'red.txt': " + e.getMessage());
                }
            }
            telemetry.update();
        }
    }
}
