package com.technototes.logger.entry;

import com.technototes.logger.Color;

import java.util.function.Supplier;

public class StringEntry extends Entry<String> {
    private String format;
    private Color entryColor;
    public StringEntry(String n, Supplier<String> s, int x, Color c, String f, Color ec) {
        super(n, s, x, c);
        format = f;
        entryColor = ec;
    }

    @Override
    public String toString() {
        return entryColor.format(format, get());
    }
}
