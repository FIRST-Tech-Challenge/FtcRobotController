package org.firstinspires.ftc.teamcode;

import java.io.IOException;
import java.nio.file.*;

import android.os.Environment;

public class FileManager {
    // The directory that save the current season's storage files.
    private static final Path seasonDirectory = Paths.get(Environment.getExternalStorage().getPath() + "/2024-2025IntoTheDeep");

    /**
     * Writes a String to a text file inside this season's directory.
     * Creates the file if it does not already exist
     *
     * @param fileName The name of the file being written to.
     * @param inputString The String that is written to the file
     * @return Whether the String was successfully written to the file.
     * @throws FileAlreadyExistsException Failed to create the output file due to it already existing.
     * @throws IOException Failed to create or write to the output file. 
     */
    public static boolean writeToFile(Path fileName, String inputString) throws FileAlreadyExistsException, IOException {
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

        Files.writeString(writeFile, inputString);
        return true;
    }

    /**
     * Read a text file inside this season's directory.
     * 
     * @param fileName
     * @return The String contained within the text file.
     *         If an error occurs, null is returned.
     * @throws IOException Failed to read the input file
     */
    public static String readFromFile(Path fileName) throws IOException {
        Path readFile = seasonDirectory.resolve(fileName);

        if (Files.exists(readFile)) {
            return Files.readString(readFile);
        }

        return null;
    }
}