package org.firstinspires.ftc.teamcode.functional;

@FunctionalInterface
public interface Provider<T> {
    T get();
}
