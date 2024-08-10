//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package android.util;

import android.annotation.NonNull;
import android.annotation.Nullable;
import androidx.annotation.RecentlyNonNull;
import androidx.annotation.RecentlyNullable;

import java.util.Map;

public final class Log {
    public static final int ASSERT = 7;
    public static final int DEBUG = 3;
    public static final int ERROR = 6;
    public static final int INFO = 4;
    public static final int VERBOSE = 2;
    public static final int WARN = 5;

    Log() {
        throw new RuntimeException("Stub!");
    }

    public static int v(@RecentlyNullable String tag, @RecentlyNonNull String msg) {
        return println(VERBOSE, tag, msg);
    }

    public static int v(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        return println(VERBOSE, tag, msg);
    }

    public static int d(@RecentlyNullable String tag, @RecentlyNonNull String msg) {
        return println(DEBUG, tag, msg);
    }

    public static int d(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        return println(DEBUG, tag, msg);
    }

    public static int i(@RecentlyNullable String tag, @RecentlyNonNull String msg) {
        return println(INFO, tag, msg);
    }

    public static int i(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        return println(INFO, tag, msg);
    }

    public static int w(@RecentlyNullable String tag, @RecentlyNonNull String msg) {
        return println(WARN, tag, msg);
    }

    public static int w(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        return println(WARN, tag, msg);
    }

    public static native boolean isLoggable(@RecentlyNullable String var0, int var1);

    public static int w(@RecentlyNullable String tag, @RecentlyNullable Throwable tr) {
        return println(DEBUG, tag, tr.toString());
    }

    public static int e(@RecentlyNullable String tag, @RecentlyNonNull String msg) {
        return println(ERROR, tag, msg);
    }

    public static int e(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        return println(ERROR, tag, msg);
    }

    public static int wtf(@RecentlyNullable String tag, @RecentlyNullable String msg) {
        return println(DEBUG, tag, msg);
    }

    public static int wtf(@RecentlyNullable String tag, @RecentlyNonNull Throwable tr) {
        return println(DEBUG, tag, tr.toString());
    }

    public static int wtf(@RecentlyNullable String tag, @RecentlyNullable String msg, @RecentlyNullable Throwable tr) {
        return println(DEBUG, tag, msg);
    }

    @NonNull
    public static String getStackTraceString(@Nullable Throwable tr) {
        return "";
    }

    public static int println(int priority, @RecentlyNullable String tag, @RecentlyNonNull String msg) {
        String priorityString =
                priority == ASSERT ? "ASSERT" :
                        priority == DEBUG ? "DEBUG" :
                                priority == ERROR ? "ERROR" :
                                        priority == INFO ? "INFO" :
                                                priority == VERBOSE ? "VERBOSE" :
                                                        priority == WARN ? "WARN" :
                                                                "UNKNOWN";
        System.out.printf("Log(%s) %s: %s\n", priorityString, tag, msg);
        return 0;
    }
}
