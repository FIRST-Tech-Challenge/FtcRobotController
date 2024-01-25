package org.firstinspires.ftc.teamcode.util.Other;

public interface DynamicTypeValue <T> {
    default T getValue() {
        return null;
    }
    default void setValue(final T value) {

    }
}
