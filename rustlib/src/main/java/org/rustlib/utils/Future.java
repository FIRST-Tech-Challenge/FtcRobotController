package org.rustlib.utils;

import java.util.function.Supplier;

public class Future<T> implements Supplier<T> {
    private final Supplier<T> supplier;
    private T path = null;

    public Future(Supplier<T> supplier) {
        this.supplier = supplier;
    }

    @Override
    public T get() {
        if (path == null) {
            path = supplier.get();
        }
        return path;
    }
}
