package com.technototes.logger.entry;


import com.technototes.logger.Color;

import java.util.function.Supplier;

public class NumberEntry extends Entry<Number> {
    protected Color numberColor;
    public NumberEntry(String n, Supplier<Number> s, int x, Color c, Color num) {
        super(n, s, x, c);
        numberColor = num;
    }
    public NumberEntry(String n, Supplier<Number> s, int x, Color c) {
        super(n, s, x, c);
        numberColor = Color.NO_COLOR;
    }

    @Override
    public String toString() {
        return numberColor.format(get());
    }
}
