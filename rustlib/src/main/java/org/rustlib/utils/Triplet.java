package org.rustlib.utils;

import java.util.Objects;

public class Triplet<T, U, V> {
    public final T first;
    public final U second;
    public final V third;

    public Triplet(T first, U second, V third) {
        this.first = first;
        this.second = second;
        this.third = third;
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Triplet)) {
            return false;
        }
        Triplet<?, ?, ?> triplet = (Triplet<?, ?, ?>) o;
        return Objects.equals(triplet.first, first) && Objects.equals(triplet.second, second) && Objects.equals(triplet.third, third);
    }
}
