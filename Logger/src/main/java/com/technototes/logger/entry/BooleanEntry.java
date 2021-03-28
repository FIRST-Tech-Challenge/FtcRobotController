package com.technototes.logger.entry;

import com.technototes.logger.Color;

import java.util.function.Supplier;

public class BooleanEntry extends Entry<Boolean> {
    private StringEntry trueEntry, falseEntry;
    public BooleanEntry(String n, Supplier<Boolean> s, int index, String wt, String wf, Color c,
                        String tf, String ff, Color tc, Color fc) {
        super(n, s, index, c);
        trueEntry = new StringEntry("", ()->wt, -1, Color.NO_COLOR, tf, tc);
        falseEntry = new StringEntry("", ()->wf, -1, Color.NO_COLOR, ff, fc);
    }

    @Override
    public String toString() {
        return (get() ? trueEntry : falseEntry).get();
    }
}
