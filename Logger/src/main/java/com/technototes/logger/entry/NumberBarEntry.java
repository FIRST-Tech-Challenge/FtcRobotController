package com.technototes.logger.entry;


import com.technototes.logger.Color;
import com.technototes.logger.Logger;

import java.util.function.Supplier;

public class NumberBarEntry extends NumberSliderEntry {
    //primary is bar color, secondary is outline, tertiary is incomplete bar
    public NumberBarEntry(String n, Supplier<Number> s, int x, Number mi, Number ma, Number sc, Color c, Color pri, Color sec, Color tert) {
        super(n, s, x, mi, ma, sc, c, pri, sec, tert);
    }

    @Override
    public String toString() {
        StringBuilder r = new StringBuilder(secondary.format("["));
        int totalAmt = (int)((max()-min())/scale());
        int fullAmt = (int)(Math.round((getDouble()-min())/scale()));
        r.append(primary.format(Logger.repeat("█", fullAmt+1)));
        r.append(tertiary.format(Logger.repeat("━", totalAmt-fullAmt)));
        r.append(secondary.format("]"));
        return r.toString();
    }

}
