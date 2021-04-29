package globalfunctions;

public class Sleep {
    public interface InterruptedExceptionRunnable {
        void run() throws InterruptedException;
    }
    public static void trySleep(InterruptedExceptionRunnable runnable) {
        try { runnable.run(); } catch (InterruptedException ignored) { }
    }
}
