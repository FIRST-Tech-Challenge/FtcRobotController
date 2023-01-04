package ftc.rogue.blacksmith.meepmeep

import com.noahbres.meepmeep.MeepMeep
import ftc.rogue.blacksmith.util.invokeMethod
import java.io.*
import java.util.*
import java.util.concurrent.TimeUnit

// ---------------------------------------------------------------------------------------------
// Note that some of the documentation is broken because of auto
// code conversion and I'm too lazy to fix it
// ---------------------------------------------------------------------------------------------

/**
 * A class to persist the state of the visualizer window.
 *
 * Basically saves the position of the [MeepMeep] GUI window so that it opens in the some position
 * that it was closed in.
 *
 *
 *  Basic usage example:
 * ```java
 * MeepMeep meepMeep = new MeepMeep(windowSize);
 *
 * // Create a persistence object linked to the MeepMeep instance
 * MeepMeepPersistence persistence = new MeepMeepPersistence(meepMeep);
 *
 * // Restore the settings from the persistence object to the MeepMeep instance
 * persistence.restore();
 * ```
 *
 * **NOTE:** The settings autosave every 2 seconds, and when MeepMeep is closed from the 'X'.
 *
 * @author KG
 *
 * @see MeepMeep
 * @see Properties
 */
@Suppress("MemberVisibilityCanBePrivate")
class MeepMeepPersistence @JvmOverloads constructor(
    private val meepMeep: MeepMeep,
    val savePeriod: Long = 1000L,
    val defaultFilePath: String = ".blacksmith/meepmeep.properties",
) {
    /**
     * The [Properties] object used to save and interpret the settings.
     */
    private val properties = Properties()

    /**
     * Injects the `Properties` instance with the saved state, then starts an auto-save thread to
     * save the settings every `savePeriod` seconds, as well as a shutdown hook to save the
     * settings when the program is closed normally (via the X button, __not__ the red stop square.)
     */
    init {
        reload()
        startPersistenceThread()

        // Add a shutdown hook to save the settings when the program is closed
        // Due to the way shutdown hooks work in Java, this is only called if the program
        // is exited 'normally', using System.exit(0). This means that this will be run
        // if the GUI is closed from the 'X' button, but it won't run if the code is killed
        // using the red 'stop' button from the IDE or something.
        Runtime.getRuntime().addShutdownHook( Thread(::save) )
    }

    /**
     * Runs the auto-save thread. The assigned runnable is called every 2 seconds, saving
     * the `MeepMeep` state every time it is called.
     */
    private fun startPersistenceThread() {
        // Utilizes a ScheduledExecutorService which launches a new thread which can run a given
        // piece of code at a fixed time.

        // The "thread" takes in a Runnable object, which can be implemented as a simple lambda
        // expression, shorthand for:
        // new Runnable() { @Override public void run() { ... } }.

        // However, in this case, it can be shortened even further, using the Method Reference
        // syntax, to '::save', casted to a Runnable. This is just referring to the save
        // function below.

        // It is called every 'savePeriod' seconds (defaulting to 2 if unspecified),
        // the time unit being specified by the TimeUnit.SECONDS parameter. It's automatically
        // killed when the program is closed.
        ScheduledMeepMeepExecutor.EXECUTOR.schedule(::save, savePeriod, TimeUnit.MILLISECONDS)
    }

    /**
     * Saves the [MeepMeep] state to the given file path.
     *
     *
     * At the moment, it only saves the window position of the [MeepMeep] GUI window.
     * However, the code is fully extensible, and other desired settings may be saved
     * to be restored.
     *
     * @param path The file path to save the state to
     */
    @JvmOverloads
    fun save(path: String = defaultFilePath) {
        ensureFileExistence(path)

        // Just saves the x and y coordinates of the MeepMeep GUI to the properties object
        val x = getWindowFrame().invokeMethod<Int>("getX")
        val y = getWindowFrame().invokeMethod<Int>("getY")

        properties.setProperty("windows_x", x.toString())
        properties.setProperty("windows_y", y.toString())

        // Persists the properties to a file, so that the state can be saved across
        // program restarts.
        BufferedWriter( FileWriter(path) ).use { writer ->
            properties.store(writer, null) // Null means no comments added to the file
        }
    }

    /**
     * Restores the [MeepMeep] state from the given file path.
     *
     *
     * At the moment, it only restores the window position of the [MeepMeep] GUI window.
     * However, the code is fully extensible, and other desired settings may be restored.
     *
     * @param path The file path to restore the state from
     */
    @JvmOverloads
    fun reload(path: String = defaultFilePath) {
        ensureFileExistence(path)

        // Loads in the properties from the given file
        BufferedReader( FileReader(path) )
            .use(properties::load)
    }

    /**
     * Restores the `MeepMeep` state from the `Properties` object.
     *
     *
     * At the moment, it only restores the window position of the [MeepMeep] GUI window.
     * However, the code is fully extensible, and other desired settings may be restored.
     *
     *
     * **Note:** This does NOT restore the settings from the persistence file, but rather from
     * whatever settings are in the `Properties` object currently.
     * [MeepMeepPersistence.reload] must be manually called first to restore the settings
     * from the persistence file <u>if the persistence file is manually changed.</u>
     */
    fun restore() {
        // Sets the x, y coords of the MeepMeep window to the values stored in the properties object
        // Needs to be converted to an int first, as the properties object stores them as strings
        val x = properties.getProperty("windows_x", "0").toInt()
        val y = properties.getProperty("windows_y", "0").toInt()

        getWindowFrame().invokeMethod<Any>("setLocation", x to Integer.TYPE, y to Integer.TYPE)
    }

    /**
     * Ensures that the given file path exists. Creates the file if it does not.
     *
     * @param path The file path to ensure exists
     */
    private fun ensureFileExistence(path: String) {
        // The .createNewFile() method creates the file if it doesn't exist,
        // and does nothing at all if it already exists
        File(path).createNewFile()
    }

    private fun getWindowFrame() = meepMeep.invokeMethod<Any>("getWindowFrame")
}
