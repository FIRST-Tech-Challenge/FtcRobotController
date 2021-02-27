package org.firstinspires.ftc.teamcode.toolkit.misc;

public class Utils {

    public static void sleep(long millis) {
        long initialTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - initialTime < millis) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
