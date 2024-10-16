package org.firstinspires.ftc.teamcode.tatooine.utils.Alliance;

import java.io.File;
import java.io.IOException;

public class CheckAlliance {
    public static boolean isRed() {
        //sets the file in code location to the path
        File redFile = new File("/sdcard/FIRST/red.txt");
        //if there is a file located in the path then returns that i am red team
        if (redFile.exists()) {
            return true;
        } else {
            return false;

        }
    }
}
