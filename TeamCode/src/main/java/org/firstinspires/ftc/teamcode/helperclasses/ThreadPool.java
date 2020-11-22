package org.firstinspires.ftc.teamcode.helperclasses;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.firstinspires.ftc.teamcode.competition.Hardware;


public class ThreadPool {

    public static Hardware robot = new Hardware();

    public static ExecutorService pool = Executors.newCachedThreadPool();

    /**
     * Shuts down the current pool to stop code from running after op mode ends
     * Then renews the thread pool so the app does not need to be restarted to make a new pool
     */
    public static void renewPool() {
        pool.shutdownNow();
        pool = Executors.newCachedThreadPool();
    }

}
