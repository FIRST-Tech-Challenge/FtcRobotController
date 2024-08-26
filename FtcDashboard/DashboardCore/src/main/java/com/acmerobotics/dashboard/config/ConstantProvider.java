package com.acmerobotics.dashboard.config;

public class ConstantProvider<T> implements ValueProvider<T> {
    private final T value;

    public ConstantProvider(T value) {
        this.value = value;
    }

    @Override
    public T get() {
        return value;
    }

    @Override
    public void set(T value) {
        throw new UnsupportedOperationException("Constants cannot be modified");
    }
}
