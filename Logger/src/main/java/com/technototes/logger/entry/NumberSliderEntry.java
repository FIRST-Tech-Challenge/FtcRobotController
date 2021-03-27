package com.technototes.logger.entry;

import com.technototes.logger.Color;
import com.technototes.logger.Logger;

import java.util.function.Supplier;

public class NumberSliderEntry extends NumberEntry {
    protected Number min, max, scale;
    protected Color primary, secondary, tertiary;
    public NumberSliderEntry(String n, Supplier<Number> s, int x, Number mi, Number ma, Number sc, Color c, Color pr, Color sec, Color tert) {
        super(n, s, x, c);
        min = mi;
        max = ma;
        scale = sc;
        primary = pr;
        secondary = sec;
        tertiary = tert;
    }
    public double min(){
        return min.doubleValue();
    }
    public double max(){
        return max.doubleValue();
    }
    public double scale(){
        return scale.doubleValue();
    }

    public double getDouble(){
        return get().doubleValue();
    }
    @Override
    public String toString() {
        StringBuilder r = new StringBuilder(secondary.format("["));
        int totalAmt = (int)((max()-min())/scale());
        int fullAmt = (int)(Math.round((getDouble()-min())/scale()));
        r.append(tertiary.format(Logger.repeat("━", fullAmt)));
        r.append(primary.format("█"));
        r.append(tertiary.format(Logger.repeat("━", totalAmt-fullAmt)));
        r.append(secondary.format("]"));
        return r.toString();
    }
}
