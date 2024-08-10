//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package android.os;

import static java.lang.System.nanoTime;

import java.io.FileDescriptor;
import java.io.IOException;
import java.util.Map;

public final class Debug {
    public static final int SHOW_CLASSLOADER = 2;
    public static final int SHOW_FULL_DETAIL = 1;
    public static final int SHOW_INITIALIZED = 4;
    /** @deprecated */
    @Deprecated
    public static final int TRACE_COUNT_ALLOCS = 1;

    Debug() {
        throw new RuntimeException("Stub!");
    }

    public static void waitForDebugger() {
        throw new RuntimeException("Stub!");
    }

    public static boolean waitingForDebugger() {
        throw new RuntimeException("Stub!");
    }

    public static boolean isDebuggerConnected() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void changeDebugPort(int port) {
        throw new RuntimeException("Stub!");
    }

    public static void startNativeTracing() {
        throw new RuntimeException("Stub!");
    }

    public static void stopNativeTracing() {
        throw new RuntimeException("Stub!");
    }

    public static void enableEmulatorTraceOutput() {
        throw new RuntimeException("Stub!");
    }

    public static void startMethodTracing() {
        throw new RuntimeException("Stub!");
    }

    public static void startMethodTracing(String tracePath) {
        throw new RuntimeException("Stub!");
    }

    public static void startMethodTracing(String tracePath, int bufferSize) {
        throw new RuntimeException("Stub!");
    }

    public static void startMethodTracing(String tracePath, int bufferSize, int flags) {
        throw new RuntimeException("Stub!");
    }

    public static void startMethodTracingSampling(String tracePath, int bufferSize, int intervalUs) {
        throw new RuntimeException("Stub!");
    }

    public static void stopMethodTracing() {
        throw new RuntimeException("Stub!");
    }

    public static long threadCpuTimeNanos() {
        return nanoTime();
    }

    /** @deprecated */
    @Deprecated
    public static void startAllocCounting() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void stopAllocCounting() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalAllocCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalAllocCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalAllocSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalAllocSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalFreedCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalFreedCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalFreedSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalFreedSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalGcInvocationCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalGcInvocationCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalClassInitCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalClassInitCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalClassInitTime() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalClassInitTime() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalExternalAllocCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalExternalAllocSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalExternalAllocCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalExternalAllocSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalExternalFreedCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalExternalFreedCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getGlobalExternalFreedSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetGlobalExternalFreedSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getThreadAllocCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetThreadAllocCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getThreadAllocSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetThreadAllocSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getThreadExternalAllocCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetThreadExternalAllocCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getThreadExternalAllocSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetThreadExternalAllocSize() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int getThreadGcInvocationCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetThreadGcInvocationCount() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static void resetAllCounts() {
        throw new RuntimeException("Stub!");
    }

    public static String getRuntimeStat(String statName) {
        throw new RuntimeException("Stub!");
    }

    public static Map<String, String> getRuntimeStats() {
        throw new RuntimeException("Stub!");
    }

    public static native long getNativeHeapSize();

    public static native long getNativeHeapAllocatedSize();

    public static native long getNativeHeapFreeSize();

    // ### public static native void getMemoryInfo(MemoryInfo var0);

    public static native long getPss();

    /** @deprecated */
    @Deprecated
    public static int setAllocationLimit(int limit) {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public static int setGlobalAllocationLimit(int limit) {
        throw new RuntimeException("Stub!");
    }

    public static void printLoadedClasses(int flags) {
        throw new RuntimeException("Stub!");
    }

    public static int getLoadedClassCount() {
        throw new RuntimeException("Stub!");
    }

    public static void dumpHprofData(String fileName) throws IOException {
        throw new RuntimeException("Stub!");
    }

    public static native int getBinderSentTransactions();

    public static native int getBinderReceivedTransactions();

    public static native int getBinderLocalObjectCount();

    public static native int getBinderProxyObjectCount();

    public static native int getBinderDeathObjectCount();

    public static boolean dumpService(String name, FileDescriptor fd, String[] args) {
        throw new RuntimeException("Stub!");
    }

// ###    public static void attachJvmtiAgent(@NonNull String library, @Nullable String options, @Nullable ClassLoader classLoader) throws IOException {
// ###        throw new RuntimeException("Stub!");
// ###    }

}
