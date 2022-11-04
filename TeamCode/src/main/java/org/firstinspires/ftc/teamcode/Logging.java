package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Map;
import java.util.Set;
import java.util.TimeZone;
import java.util.logging.ConsoleHandler;
import java.util.logging.FileHandler;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

/**
 * Custom logging class. Configures log system (not console) to write to a disk file.
 * Customize the setup() method for the controller phone you are using.
 */
public class Logging
{
    /**
     * PrintStream that writes to out custom logging location.
     */
    public static final PrintStream	logPrintStream = new PrintStream(new LoggingOutputStream());

    /**
     * Used by other classes to implement logging. Other classes should log with methods on this
     * object or the simpler methods included in this class. The methods in class record the
     * location in your program where you call them.
     */
    public final static Logger logger = Logger.getGlobal();

    // The log file can be copied from the ZTE robot controller to your PC for review by using the
    // following AndroidDeBugger command in the Android Studio Terminal window:
    //
    // ZTE: adb pull //storage/sdcard0/Logging.txt c:\temp\robot_logging.txt
    // MOTO G: adb pull sdcard/Logging.txt c:\temp\robot_logging.txt
    // Control Hub: adb pull sdcard/Logging.txt c:\temp\robot_Logging.txt

    /**
     * Indicates if logging is turned on or off.
     */
    public static boolean enabled = true;

    private static FileHandler 		fileTxt;
    //static private SimpleFormatter	formatterTxt;
    private static LogFormatter		logFormatter;
    private static boolean          isSetup;

    /**
     * Configures our custom logging. If you don't use this custom logging, logging will go to
     * the default logging location, typically the console. Call setup to turn on custom logging.
     * With custom logging turned on logging goes to the console and also to the file opened in
     * setup().
     */

    /**
     * Call to initialize our custom logging system.
     * @throws IOException
     */
    static public void setup() throws IOException
    {
        if (isSetup)
        {
            logger.info("========================================================================");
            return;
        }

        // get the global logger to configure it and add a file handler.
        Logger logger = Logger.getGlobal();

        logger.setLevel(Level.ALL);

        // If we decide to redirect system.out to our log handler, then following
        // code will delete the default log handler for the console to prevent
        // a recursive loop. We would only redirect system.out if we only want to
        // log to the file. If we delete the console handler we can skip setting
        // the formatter...otherwise we set our formatter on the console logger.

        Logger rootLogger = Logger.getLogger("");

        Handler[] handlers = rootLogger.getHandlers();

//            if (handlers[0] instanceof ConsoleHandler)
//            {
//                rootLogger.removeHandler(handlers[0]);
//                return;
//            }

        logFormatter = new LogFormatter();

        // Set our formatter on the console log handler.
        if (handlers[0] instanceof ConsoleHandler) handlers[0].setFormatter(logFormatter);

        // Now create a handler to log to a file on controller phone "disk".

        // For ZTE:
        //fileTxt = new FileHandler("storage/sdcard0/Logging.txt", 0 , 1);

        // For MOTO G:
        fileTxt = new FileHandler("sdcard/Logging.txt", 0 , 1);

        fileTxt.setFormatter(logFormatter);



        logger.addHandler(fileTxt);

        isSetup = true;
    }

    /**
     * Flush logged data to disk file. Not normally needed.
     */
    public static void flushlog()
    {
        fileTxt.flush();
    }

    // Our custom formatter for logging output.

    private static class LogFormatter extends Formatter
    {
        public String format(LogRecord rec)
        {
            StringBuffer buf = new StringBuffer(1024);

            buf.append(String.format("<%d>", rec.getThreadID())); //Thread.currentThread().getId()));
            buf.append(formatDate(rec.getMillis()));
            buf.append(" ");
            buf.append(formatMessage(rec));
            buf.append("\r\n");

            return buf.toString();
        }

        private String formatDate(long milliseconds)
        {
            SimpleDateFormat dateFormat = new SimpleDateFormat("hh:mm:ss:SSS");
            dateFormat.setTimeZone(TimeZone.getTimeZone("America/Los_Angeles"));
            Date resultDate = new Date(milliseconds);
            return dateFormat.format(resultDate);
        }
    }

    /**
     * Log blank line with program location.
     */
    public static void log()
    {
        if (!enabled) return;

        logger.log(Level.INFO, currentMethod(2));
    }

    /**
     * Log message with optional formatting and program location.
     * @param message message with optional format specifiers for listed parameters
     * @param parms parameter list matching format specifiers
     */
    public static void log(String message, Object... parms)
    {
        if (!enabled) return;

        logger.log(Level.INFO, String.format(" RENO - %s: %s", currentMethod(2), String.format(message, parms)));
    }

    /**
     * Log message with optional formatting and no program location.
     * @param message message with optional format specifiers for listed parameters
     * @param parms parameter list matching format specifiers
     */
    public static void logNoMethod(String message, Object... parms)
    {
        if (!enabled) return;

        logger.log(Level.INFO, String.format(message, parms));
    }

    /**
     * Log message with no formatting and program location.
     * @param message message with optional format specifiers for listed parameters
     */
    public static void logNoFormat(String message)
    {
        if (!enabled) return;

        logger.log(Level.INFO, String.format("%s: %s", currentMethod(2), message));
    }

    /**
     * Log message with no formatting and no program location.
     * @param message message with optional format specifiers for listed parameters
     */
    public static void logNoFormatNoMethod(String message)
    {
        if (!enabled) return;

        logger.log(Level.INFO, message);
    }

    /**
     * Returns program location where call to this method is located.
     */
    public static String currentMethod()
    {
        return currentMethod(2);
    }

    private static String currentMethod(Integer level)
    {
        StackTraceElement stackTrace[];

        stackTrace = new Throwable().getStackTrace();

        try
        {
            return stackTrace[level].toString().split("teamcode.")[1];
        }
        catch (Exception e)
        {
            try
            {
                return stackTrace[level].toString().split("lib.")[1];
            }
            catch (Exception e1)
            {
                try
                {
                    return stackTrace[level].toString().split("activities.")[1];
                }
                catch (Exception e2) {e2.printStackTrace(); return "";}
            }
        }
    }

    /**
     * Write a list of configured hardware devices to the log file. Can be called in init()
     * function or later.
     * @param map hardwareMap object.
     */
    public static void logHardwareDevices(HardwareMap map)
    {
        log();

        // This list must be manually updated when First releases support for new devices.

        logDevices(map.dcMotorController);
        logDevices(map.dcMotor);
        logDevices(map.servoController);
        logDevices(map.servo);
        //logDevices(map.deviceInterfaceModule);
        logDevices(map.analogInput);
        //logDevices(map.analogOutput);
        logDevices(map.digitalChannel);
        logDevices(map.pwmOutput);
        logDevices(map.accelerationSensor);
        logDevices(map.colorSensor);
        logDevices(map.compassSensor);
        logDevices(map.gyroSensor);
        logDevices(map.irSeekerSensor);
        logDevices(map.i2cDevice);
        logDevices(map.led);
        logDevices(map.lightSensor);
        logDevices(map.opticalDistanceSensor);
        logDevices(map.touchSensor);
        logDevices(map.ultrasonicSensor);
        //logDevices(map.legacyModule);
    }

    @SuppressWarnings("unchecked")

    private static void logDevices(HardwareMap.DeviceMapping deviceMap)
    {
        for (Map.Entry<String, HardwareDevice> entry :(Set<Map.Entry<String,HardwareDevice>>) deviceMap.entrySet())
        {
            HardwareDevice device = entry.getValue();
            log("%s;%s;%s", entry.getKey(), device.getDeviceName(), device.getConnectionInfo());
        }
    }

    /**
     * Get the user assigned name for a hardware device.
     * @param deviceMap The DEVICE_TYPE map, such as hardwareDevice.dcMotor, that the dev belongs to.
     * @param dev Instance of a device of DEVICE_TYPE.
     * @return User assigned name or empty string if not found.
     */

    @SuppressWarnings("unchecked")

    public static String getDeviceUserName(HardwareMap.DeviceMapping deviceMap, HardwareDevice dev)
    {
        for (Map.Entry<String, HardwareDevice> entry : (Set<Map.Entry<String,HardwareDevice>>) deviceMap.entrySet())
        {
            HardwareDevice device = entry.getValue();
            if (dev == device) return entry.getKey();
        }

        return "";
    }

    // An output stream that writes to our logging system. Writes data with flush on
    // flush call or on a newline character in the stream.

    private static class LoggingOutputStream extends OutputStream
    {
        private static final int	DEFAULT_BUFFER_LENGTH = 2048;
        private boolean 			hasBeenClosed = false;
        private byte[] 				buf;
        private int 				count, curBufLength;

        public LoggingOutputStream()
        {
            curBufLength = DEFAULT_BUFFER_LENGTH;
            buf = new byte[curBufLength];
            count = 0;
        }

        public void write(final int b) throws IOException
        {
            if (!enabled) return;

            if (hasBeenClosed) {throw new IOException("The stream has been closed.");}

            // don't log nulls
            if (b == 0) return;

            // force flush on newline character, dropping the newline.
            if ((byte) b == '\n')
            {
                flush();
                return;
            }

            // would this be writing past the buffer?
            if (count == curBufLength)
            {
                // grow the buffer
                final int newBufLength = curBufLength + DEFAULT_BUFFER_LENGTH;
                final byte[] newBuf = new byte[newBufLength];
                System.arraycopy(buf, 0, newBuf, 0, curBufLength);
                buf = newBuf;
                curBufLength = newBufLength;
            }

            buf[count] = (byte) b;

            count++;
        }

        public void flush()
        {
            if (count == 0) return;

            final byte[] bytes = new byte[count];

            System.arraycopy(buf, 0, bytes, 0, count);

            String str = new String(bytes);

            logNoFormatNoMethod(str);

            count = 0;
        }

        public void close()
        {
            flush();

            hasBeenClosed = true;
        }
    }
}
