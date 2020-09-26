package org.firstinspires.ftc.teamcode.support;

import android.util.Log;

/**
 * Provides basic logging support by wrapping <code>android.util.Log</code> calls
 *  to add lazy message formatting and log level threshold support.
 * Typical usage is to extend this class:
 * <pre>
 * public class MyClass extends Logger&lt;MyClass&gt; {
 *     ...
 *     // within some method
 *     verbose("String.format pattern", arg1, arg2, ...);
 *     ...
 * }
 *
 * // in a container class
 * MyClass instance = new MyClass().configureLogging("some tag", Log.VERBOSE);
 * </pre>
 * An alternative is to instantiate it directly, store it in a (member) variable
 *  and manually invoke <code>configureLogging()</code> prior to use.
 */
public class Logger<T> {
    protected String logTag = getClass().getSimpleName(); // tag to use for logging
    protected int logLevel = Log.INFO; // log level to use

    /**
     * Configures logging to use given tag and log level
     * @param tag short tag to distinguish log messages produced by this class
     * @param level log level threshold. Logging calls with levels below this threshold will be ignored
     * @return this class instance to support method chaining
     * @see Log
     */
    @SuppressWarnings("unchecked")
    public T configureLogging(String tag, int level) {
        this.logTag = tag;
        this.logLevel = level;
        return (T) this;
    }

    /**
     * Logs a message using <code>Log.VERBOSE</code> level.
     * Call will be ignored if currently set log level is above <code>Log.VERBOSE</code>
     * @param format message format as used by <code>String.format()</code>
     * @param args arguments to pass to <code>String.format()</code>
     */
    public void verbose(String format, Object... args) {
        if (logLevel > Log.VERBOSE || format==null) return;
        Log.v(logTag, String.format(format, args));
    }

    /**
     * Logs a message using <code>Log.DEBUG</code> level.
     * Call will be ignored if currently set log level is above <code>Log.DEBUG</code>
     * @param format message format as used by <code>String.format()</code>
     * @param args arguments to pass to <code>String.format()</code>
     */
    public void debug(String format, Object... args) {
        if (logLevel > Log.DEBUG || format==null) return;
        Log.d(logTag, String.format(format, args));
    }

    /**
     * Logs a message using <code>Log.INFO</code> level.
     * Call will be ignored if currently set log level is above <code>Log.INFO</code>
     * @param format message format as used by <code>String.format()</code>
     * @param args arguments to pass to <code>String.format()</code>
     */
    public void info(String format, Object... args) {
        if (logLevel > Log.INFO || format==null) return;
        Log.i(logTag, String.format(format, args));
    }

    /**
     * Logs a message using <code>Log.WARN</code> level.
     * Call will be ignored if currently set log level is above <code>Log.WARN</code>
     * @param format message format as used by <code>String.format()</code>
     * @param args arguments to pass to <code>String.format()</code>
     */
    public void warn(String format, Object... args) {
        if (logLevel > Log.WARN) return;
        Object lastArgument = args.length > 0 ? args[args.length - 1] : null;
        if (lastArgument instanceof Throwable) {
            Log.w(logTag, format==null ? "" : String.format(format, args), (Throwable) lastArgument);
        } else if (format!=null) {
            Log.w(logTag, String.format(format, args));
        }
    }

    /**
     * Logs a message using <code>Log.ERROR</code> level.
     * Call will be ignored if currently set log level is above <code>Log.ERROR</code>
     * @param format message format as used by <code>String.format()</code>
     * @param args arguments to pass to <code>String.format()</code>
     */
    public void error(String format, Object... args) {
        if (logLevel > Log.ERROR) return;
        Object lastArgument = args.length > 0 ? args[args.length - 1] : null;
        if (lastArgument instanceof Throwable) {
            Log.e(logTag, format==null ? "" : String.format(format, args), (Throwable) lastArgument);
        } else if (format!=null) {
            Log.e(logTag, String.format(format, args));
        }
    }
}
