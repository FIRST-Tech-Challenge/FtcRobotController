package globalfunctions;

public class Sleep {
    //Interface for runnable
    public interface InterruptedExceptionRunnable {
        void run() throws InterruptedException;
    }
    //Method to sleep for certain time
    public static void trySleep(InterruptedExceptionRunnable runnable) {
        try { runnable.run(); } catch (InterruptedException ignored) { }
    }
}
