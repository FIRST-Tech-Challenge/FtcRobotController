package com.acmerobotics.dashboard.config.reflection;

import com.acmerobotics.dashboard.config.ValueProvider;
import java.lang.reflect.Field;

/**
 * Value provider backed by a class field.
 *
 * @param <T> type of the class field
 */
public class FieldProvider<T> implements ValueProvider<T> {
    private final Field field;
    private final Object parent;

    public FieldProvider(Field field, Object parent) {
        this.field = field;
        this.parent = parent;
    }

    @SuppressWarnings("unchecked")
    @Override
    public T get() {
        try {
            return (T) field.get(parent);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void set(T value) {
        try {
            field.set(parent, value);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }
}
