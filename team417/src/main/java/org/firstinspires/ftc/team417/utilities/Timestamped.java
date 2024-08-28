package org.firstinspires.ftc.team417.utilities;

import androidx.annotation.NonNull;

public class Timestamped<E> {
    public E object;
    public double timestamp;

    public Timestamped(E object, double timestamp) {
        this.object = object;
        this.timestamp = timestamp;
    }

    @NonNull
    @Override
    public String toString() {
        return "{" + timestamp + ": " + object.toString() + "}";
    }
}