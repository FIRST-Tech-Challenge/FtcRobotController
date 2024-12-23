package org.firstinspires.ftc.teamcode.tatooine.utils.Alliance;

import java.io.File;
import java.io.IOException;

/**
 * Utility class for determining if the robot should operate in Red Alliance mode.
 * It checks the existence of a file named {@code red.txt} on the device storage.
 */
public class CheckAlliance {

    // The path to the file used to indicate Red Alliance.
    private static final String RED_FILE_PATH = "/sdcard/FIRST/red.txt";

    /**
     * Checks if the red file exists. If it does, returns {@code true} (Red Alliance).
     * Otherwise, returns {@code false} (Blue Alliance).
     *
     * @return {@code true} if {@code red.txt} is found, {@code false} otherwise.
     */
    public static boolean isRed() {
        File redFile = new File(RED_FILE_PATH);
        return redFile.exists();
    }
}
