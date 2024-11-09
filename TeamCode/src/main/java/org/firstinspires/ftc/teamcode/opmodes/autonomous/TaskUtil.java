package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Log;

import java.util.Arrays;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class TaskUtil {
    private static final String LOG_TAG = TaskUtil.class.getSimpleName();

    private static final ExecutorService taskExecutor = Executors.newCachedThreadPool();

    public static void runInParallel(Runnable... tasks) {
        CountDownLatch latch = new CountDownLatch(tasks.length);
        Arrays.stream(tasks).forEach( task -> {
            taskExecutor.submit(() -> {
                try {
                    task.run();
                } finally {
                    latch.countDown();
                }
            });
        });

        try {
            latch.await();
        } catch (InterruptedException e) {
            Log.e(LOG_TAG, String.format("runInParallel get interrupted. remaining/total: %d/%d", latch.getCount(), tasks.length));
            throw new RuntimeException(e);
        }
    }

}
