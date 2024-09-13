package org.firstinspires.ftc.teamcode;

import android.os.Environment;

public class FileManager {
    /* 
     * The directory that save the current season's storage files.
    // The current String is a placeholder,
    // and is intended to be replaced with "{start year}-{end year}{season name }"
    */
    private static final String rootDirectory = "chsRobotix";

    /**
     * Writes a given String to a file 
     * 
     * @param fileName
     * @param inputString
     * @return
     */
    public static boolean writeToFile(String fileName, String inputString) {
        String filePath = Environment.getExternalStorage().getPath
    }
}