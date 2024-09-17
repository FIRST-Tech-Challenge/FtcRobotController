package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import java.nio.file;
import java.nio.file.Path;

public class FileManager {
    /*
     * The directory that save the current season's storage files.
     * The current String is a placeholder,
     * and is intended to be replaced with the name of the current year's
     * repository.
     */
    private static final Path seasonDirectory = Paths.get(
            Environment.getExternalStorage().getPath() + "/chsRobotix");

    /**
     * Writes a String to a text file inside this season's directory.
     *
     * @param fileName
     * @param inputString
     * @return Whether the String was successfully written to the file.
     */
    public static boolean writeToFile(Path fileName, String inputString) {
        // If this season's directory does not exist, create it.
        if (!Files.exists(seasonDirectory)) {
            Files.createDirectory(seasonDirectory);
        }

        Path writeFile = seasonDirectory.resolve(fileName);
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

    /**
     * Read a text file inside this season's directory.
     * 
     * @param fileName
     * @return The String contained within the text file.
     *         If an error occurs, null is returned.
     */
    public static String readFromFile(Path fileName) {
        Path readFile = seasonDirectory.resolve(fileName);

        if (Files.exists(readFile)) {
            return Files.readString(readFile);
        }

        return null;
    }
}