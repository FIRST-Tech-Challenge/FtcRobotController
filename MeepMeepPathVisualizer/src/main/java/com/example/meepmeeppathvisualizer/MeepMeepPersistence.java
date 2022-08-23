package com.example.meepmeeppathvisualizer;

import com.noahbres.meepmeep.MeepMeep;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Properties;
import java.util.concurrent.TimeUnit;

/**
 * A class to persist the state of the visualizer window.
 * <p>Basically saves the position of the {@link MeepMeep} GUI window so that it opens in the some position
 * that it was closed in.
 *
 * <p> Basic usage example:
 * <pre>{@code MeepMeep meepMeep = new MeepMeep(windowSize);
 *
 * // Create a persistence object linked to the MeepMeep instance
 * MeepMeepPersistence persistence = new MeepMeepPersistence(meepMeep);
 *
 * // Restore the settings from the persistence object to the MeepMeep instance
 * persistence.restore();}</pre>
 *
 * <p><b>NOTE:</b> The settings autosave every 2 seconds, and when MeepMeep is closed from the 'X'.
 *
 * @author KG
 * @see MeepMeep
 * @see Properties
 */
public class MeepMeepPersistence {
    /**
     * The default file path for the persistence file. Should be set to suit the user's primary
     * desired file path.
     */
    private String defaultFilePath;

    /**
     * The {@link Properties} object used to save and interpret the settings.
     */
    private final Properties properties;

    /**
     * The {@link MeepMeep} instance to persist the settings of, or to restore settings to.
     */
    private final MeepMeep meepMeep;

    /**
     * Constructor for the {@code MeepMeepPersistence} class.
     *
     * <p>Starts an auto-save thread to save the settings every 2 seconds,
     * as well as a shutdown hook to save the settings when the program is closed.
     *
     * @param meepMeep The linked {@link MeepMeep} instance
     * @param defaultFilePath The file path to initially load state from
     */
    public MeepMeepPersistence(MeepMeep meepMeep, String defaultFilePath) {
        this.properties = new Properties();
        this.defaultFilePath = defaultFilePath;
        this.meepMeep = meepMeep;

        reload();

        startPersistenceThread();

        // Add a shutdown hook to save the settings when the program is closed
        // Due to the way shutdown hooks work in Java, this is only called if the program
        // is exited 'normally', using System.exit(0). This means that this will be run
        // if the GUI is closed from the 'X' button, but it won't run if the code is killed
        // using the red 'stop' button from the IDE or something.
        Runtime.getRuntime().addShutdownHook(
                new Thread(this::save)
        );
    }

    /**
     * Constructor for the {@link MeepMeepPersistence} class that uses the default file path.
     *
     * @param meepMeep The linked {@link MeepMeep} instance
     */
    public MeepMeepPersistence(MeepMeep meepMeep) {
        this(meepMeep, "./meepmeep.properties");
    }

    /**
     * Runs the auto-save thread. The assigned runnable is called every 2 seconds, saving
     * the {@code MeepMeep} state every time it is called.
     */
    private void startPersistenceThread() {
        // Basically utilizes a ScheduledExecutorService which launches a new thread which
        // can run a given piece of code at a fixed time.

        // The "thread" takes in a Runnable object, which can be implemented as a simple lambda
        // expression, shorthand for:
        // new Runnable() { @Override public void run() { ... } }.

        // However, in this case, it can be shortened even further, using the Method Reference
        // syntax, to 'this::save', casted to a Runnable. This is just referring to the save
        // function below.

        // It is called every 2 seconds, specified by the TimeUnit.SECONDS parameter.
        // It's automatically killed when the program is closed.
        ScheduledMeepMeepExecutor.executor.schedule(
                (Runnable) this::save, 2L, TimeUnit.SECONDS
        );
    }

    /**
     * Default method for {@link MeepMeepPersistence#save(String) save(String path)}, using the default file path.
     */
    public void save() {
        save(defaultFilePath);
    }

    /**
     * Saves the {@link MeepMeep} state to the given file path.
     *
     * <p>At the moment, it only saves the window position of the {@link MeepMeep} GUI window.
     * However, the code is fully extensible, and other desired settings may be saved
     * to be restored.
     *
     * @param path The file path to save the state to
     */
    public void save(String path) {
        ensureFileExistence(path);

        // Just saves the x and y coordinates of the MeepMeep GUI to the properties object
        properties.setProperty("windows_x", String.valueOf(meepMeep.getWindowFrame().getX()));
        properties.setProperty("windows_y", String.valueOf(meepMeep.getWindowFrame().getY()));

        // Persists the properties to a file, so that the state can be saved across
        // program restarts.
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(path))) {
            properties.store(writer, null); // Null means no comments added to the file
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    /**
     * Default method for {@link MeepMeepPersistence#reload(String) reload(String path)}, using the default file path.
     */
    public void reload() {
        reload(defaultFilePath);
    }

    /**
     * Reloads the {@code Properties} object from the given file path.
     *
     * @param path The file path to save the state to
     */
    public void reload(String path) {
        ensureFileExistence(path);

        // Loads in the properties from the given file
        try (BufferedReader reader = new BufferedReader(new FileReader(path))) {
            properties.load(reader);
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    /**
     * Restores the {@code MeepMeep} state from the {@code Properties} object.
     *
     * <p>At the moment, it only restores the window position of the {@link MeepMeep} GUI window.
     * However, the code is fully extensible, and other desired settings may be restored.
     *
     * <p><b>Note:</b> This does NOT restore the settings from the persistence file, but rather from
     * whatever settings are in the {@code Properties} object currently.
     * {@link MeepMeepPersistence#reload()} must be manually called first to restore the settings
     * from the persistence file <u>if the persistence file is manually changed.</u>
     */
    public void restore() {
        // Sets the x, y coords of the MeepMeep window to the values stored in the properties object
        // Needs to be converted to an int first, as the properties object stores them as strings
        meepMeep.getWindowFrame().setLocation(
                Integer.parseInt(properties.getProperty("windows_x", "0")),
                Integer.parseInt(properties.getProperty("windows_y", "0"))
        );
    }

    /**
     * Ensures that the given file path exists. Creates the file if it does not.
     *
     * @param path The file path to ensure exists
     */
    @SuppressWarnings("ResultOfMethodCallIgnored")
    private void ensureFileExistence(String path) {
        // The .createNewFile() method creates the file if it doesn't exist,
        // and does nothing at all if it already exists
        try {
            new File(path).createNewFile();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Gets the default file path.
     *
     * @return The default file path
     */
    public String getDefaultFilePath() {
        return defaultFilePath;
    }

    /**
     * Sets the default file path.
     *
     * @param defaultFilePath The new default file path
     */
    public void setDefaultFilePath(String defaultFilePath) {
        this.defaultFilePath = defaultFilePath;
    }
}