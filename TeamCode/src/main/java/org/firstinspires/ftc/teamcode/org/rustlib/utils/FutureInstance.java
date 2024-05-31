package org.firstinspires.ftc.teamcode.org.rustlib.utils;

import java.util.function.Supplier;

public class FutureInstance<T> implements Supplier<T> {
    private final Supplier<T> supplier;
    private T path = null;

    public FutureInstance(Supplier<T> supplier) {
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
