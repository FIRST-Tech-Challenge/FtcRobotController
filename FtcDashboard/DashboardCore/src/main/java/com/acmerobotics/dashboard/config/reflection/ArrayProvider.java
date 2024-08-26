package com.acmerobotics.dashboard.config.reflection;

import com.acmerobotics.dashboard.config.ValueProvider;
import java.lang.reflect.Array;
import java.lang.reflect.Field;

public class ArrayProvider<T> implements ValueProvider<T> {
    private final Field field;
    private final Object parent;
    private final int[] indices;

    public ArrayProvider(Field field, Object parent, int... indices) {
        this.field = field;
        this.parent = parent;
        this.indices = indices;
    }

    @SuppressWarnings("unchecked")
    @Override
    public T get() {
        try {
            return (T) getArrayRecursive(field.get(parent), indices);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        } catch (ArrayIndexOutOfBoundsException e) {
            return null;
        }
    }

    @Override
    public void set(T value) {
        try {
            setArrayRecursive(field.get(parent), value, indices);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        } catch (ArrayIndexOutOfBoundsException ignored) {
        }
    }


    public static Object getArrayRecursive(Object object, int[] indices)
        throws ArrayIndexOutOfBoundsException, IllegalAccessException {
        for (int index : indices) {
            object = Array.get(object, index);
        }
        return object;
    }

    public static void setArrayRecursive(Object object, Object value, int[] indices) {
        for (int i = 0; i < indices.length - 1; i++) {
            object = Array.get(object, indices[i]);
        }
        Array.set(object, indices[indices.length - 1], value);
    }
}
