package org.darbots.darbotsftclib.libcore.debug;

public class Assertions {
    public static boolean isDoubleValid(double number){
        return !(Double.isNaN(number) || Double.isInfinite(number));
    }
}
