package org.firstinspires.ftc.teamcode.subsystems.swerve;

import androidx.annotation.NonNull;

public class Triplet<T,U,V> {
    private final T first;
    private final U second;
    private final V third;

    public Triplet(T first, U second, V third) {
        this.first = first;
        this.second = second;
        this.third = third;
    }

    public T getFirst() { return first; }
    public U getSecond() { return second; }
    public V getThird() { return third; }
    @NonNull
    @Override
    public String toString() {
        return first.toString() + ", " + second.toString() + ", " + third.toString();
    }
}
