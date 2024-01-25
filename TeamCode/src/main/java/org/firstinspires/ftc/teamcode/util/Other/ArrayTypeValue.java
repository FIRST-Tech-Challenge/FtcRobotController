package org.firstinspires.ftc.teamcode.util.Other;

import java.lang.reflect.Array;

public class ArrayTypeValue<T> implements DynamicTypeValue<T> {
    private T[] values;

    public ArrayTypeValue(T[] values) {
        this.values = values;
    }

    public T get(int index) {
        return values[index];
    }

    public void set(int index, T value) {
        values[index] = value;
    }
}
