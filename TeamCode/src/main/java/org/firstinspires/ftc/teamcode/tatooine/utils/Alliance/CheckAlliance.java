package org.firstinspires.ftc.teamcode.tatooine.utils.Alliance;

import java.io.File;
import java.io.IOException;

public class CheckAlliance {
    public static boolean isRed() {
        File redFile = new File("/sdcard/FIRST/red.txt");
        if (redFile.exists()) {
            return true;
        } else {
            return false;

        }
    }
}
