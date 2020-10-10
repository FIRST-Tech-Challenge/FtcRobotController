package com.technototes.logger.entry;

import java.util.function.Supplier;

public class StringEntry extends Entry<String> {
    public StringEntry(String n, Supplier<String> s, int x) {
        super(n, s, x);
    }

    @Override
    public String toString() {
        return super.toString();
    }
}
