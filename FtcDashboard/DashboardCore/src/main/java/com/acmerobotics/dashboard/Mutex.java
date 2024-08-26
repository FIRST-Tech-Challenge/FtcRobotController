package com.acmerobotics.dashboard;

// reentrant
public final class Mutex<T> {
    private final T obj;

    public Mutex(T obj) {
        this.obj = obj;
    }

    public interface Fun<T, V> {
        V apply(T obj);
    }

    public interface UnitFun<T> {
        void apply(T obj);
    }

    // There's no guarantee against obj leaking, though any such occurrences will stand out.
    public synchronized <V> V with(Fun<T, V> f) {
        return f.apply(obj);
    }

    public synchronized void with(UnitFun<T> f) {
        f.apply(obj);
    }
}
