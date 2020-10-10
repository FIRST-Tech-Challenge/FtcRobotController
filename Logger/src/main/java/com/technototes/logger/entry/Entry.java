package com.technototes.logger.entry;

import java.util.function.Supplier;

public abstract class Entry<T> implements Supplier<T> {

    public int x;
    public Supplier<T> supplier;
    public String name;

    public Entry(String n, Supplier<T> s, int index){
        x = index;
        supplier = s;
        name = n;
    }

    @Override
    public T get() {
        return supplier.get();
    }

    @Override
    public String toString() {
        return name+": ["+get()+"]";
    }
}
