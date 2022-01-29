package com.SCHSRobotics.HAL9001.util.misc;

import android.util.Log;

import com.SCHSRobotics.HAL9001.util.exceptions.DumpsterFireException;

import org.jetbrains.annotations.NotNull;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A static class that contains functions useful for dealing with files.
 * <p>
 * Creation Date: 9/11/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.1.0
 */
public class HALFileUtil {

    /**
     * A private constructor for HALFileUtil. Used to make the class static.
     */
    private HALFileUtil() {
    }

    /**
     * Gets all the file names in a given directory path. Has the option to exclude unwanted filenames.
     *
     * @param directoryPath The path to the directory to search.
     * @param excluding The filenames to exclude from the search.
     * @return An array of all the filenames in the directory (except excluded filenames).
     */
    @NotNull
    public static String[] getAllFileNames(String directoryPath, String... excluding) {
        File configDirectory = new File(directoryPath);
        File[] allFiles = configDirectory.listFiles();
        List<String> filenames = new ArrayList<>();
        for (File allFile : allFiles) {
            String filename = allFile.getName().replace(".txt", "");

            boolean notInExcluded = true;
            for (String excludedFilename : excluding)
                notInExcluded &= !filename.equals(excludedFilename);
            if (notInExcluded) filenames.add(filename);
        }

        Collections.sort(filenames);
        String[] filenamesArray = new String[filenames.size()];
        filenames.toArray(filenamesArray);

        return filenamesArray;
    }

    /**
     * Deletes the file at the given path.
     *
     * @param filepath The path to the file being deleted.
     */
    public static void deleteFile(String filepath) {
        File fileToDelete = new File(filepath);
        if (!fileToDelete.delete()) {
            Log.e("File Issues", "Problem deleting file at " + filepath);
        }
    }

    /**
     * Creates an empty file at the given path.
     *
     * @param filepath The path to the file being created.
     * @throws DumpsterFireException Throws this exception if there was an IO error creating the file.
     */
    public static void createFile(String filepath) {
        File fileToCreate = new File(filepath);

        if (!fileToCreate.exists()) {
            try {
                if (!fileToCreate.createNewFile()) {
                    Log.e("File Error", "Could not create file at " + filepath);
                }
            } catch (IOException e) {
                throw new DumpsterFireException("Error creating file at " + filepath);
            }
        }
    }

    /**
     * Creates a directory at the given path.
     *
     * @param directoryPath The path to the directory to create.
     */
    public static void createDirectory(String directoryPath) {
        File directory = new File(directoryPath);
        if (!directory.exists()) {
            Log.i("File Creation", directory.mkdir() ? "Directory created! Path: " + directoryPath : "File error, couldn't create directory at " + directoryPath);
        }
    }

    /**
     * Gets whether the file at the given path exists.
     *
     * @param filepath The filepath to check.
     * @return Whether there is a file at the specified path.
     */
    public static boolean fileExists(String filepath) {
        File file = new File(filepath);
        return file.exists();
    }

    /**
     * Saves an object in a file at the given filepath.
     *
     * @param filepath The path to the file to save that object.
     * @param data The object to save in that file.
     */
    public static void saveObject(String filepath, Object data) {
        FileOutputStream fos;
        try {

            File file = new File(filepath);
            if (file.exists()) {
                deleteFile(filepath);
                createFile(filepath);
            }

            fos = new FileOutputStream(filepath, true);

            ObjectOutputStream fWriter;

            try {
                fWriter = new ObjectOutputStream(fos);

                fWriter.writeObject(data);

                fWriter.flush();
                fWriter.close();
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                fos.getFD().sync();
                fos.close();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Saves a string of data to a file at the given filepath.
     *
     * @param filepath The path to the file to save the data at.
     * @param data     The data to save.
     */
    public static void save(String filepath, String data) {
        FileOutputStream fos;
        try {

            File file = new File(filepath);
            if (file.exists()) {
                deleteFile(filepath);
                createFile(filepath);
            }

            fos = new FileOutputStream(filepath, true);

            FileWriter fWriter;

            try {
                fWriter = new FileWriter(fos.getFD());

                fWriter.write(data);

                fWriter.flush();
                fWriter.close();
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                fos.getFD().sync();
                fos.close();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Reads the file at the given filepath into a string.
     *
     * @param filepath The path to the file to read.
     * @return The data contained in the file.
     */
    public static String readFile(String filepath) {
        FileInputStream fis;
        StringBuilder output = new StringBuilder();

        try {
            fis = new FileInputStream(filepath);

            FileReader fReader;
            BufferedReader bufferedReader;

            try {
                fReader = new FileReader(fis.getFD());
                bufferedReader = new BufferedReader(fReader);

                String line;
                while((line = bufferedReader.readLine()) != null) {
                    output.append(line);
                    output.append('\n');
                }

                bufferedReader.close();
                fReader.close();
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                fis.getFD().sync();
                fis.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        if (output.length() > 0) return output.substring(0, output.length() - 1);
        return "";
    }
}
