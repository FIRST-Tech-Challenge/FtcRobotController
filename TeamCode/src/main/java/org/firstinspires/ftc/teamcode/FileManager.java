package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import android.os.Environment;

public class FileManager {
    /* 
     * The directory that save the current season's storage files.
    // The current String is a placeholder,
    // and is intended to be replaced with the name of the current year's repository.
    */
    private static final Path seasonDirectory = Paths.get(Environment.getExternalStorage().getPath() + "/chsRobotix");

    /**
     * Writes a given String to a given file within seasonDirectory
     * 
     * @param fileName
     * @param inputString
     * @return Whether the String was successfully written to the file. 
     */
    public static boolean writeToFile(String fileName, String inputString) {
        // If this season's directory does not exist, create it.
        if (!Files.exists(seasonDirectory)) {
            Files.createDirectory(seasonDirectory);
        }

        Path writeFile = Paths.get(seasonDirectory, fileName);
        try {
            // Create the file if it does not exist
            Files.createFile(writeFile);

        } catch (FileAlreadyExistsException err) { 
            // The file already existing is not a problem.

        } catch (IOException err) {
            return false;
        }

        Files.write(writeFile, inputString);
        return true;
    }

    public static String readFromFile(String fileName) {
        return true;
    }
}