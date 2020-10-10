package org.firstinspires.ftc.teamcode.utility;

public class Timing {
    public static void delay(long delay){
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start < delay){
            //wait
        }
    }
}
