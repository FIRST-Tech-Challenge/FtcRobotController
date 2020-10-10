package com.technototes.library.measurement.ratio;

import com.technototes.library.measurement.unit.Unit;

public class SameTypeRatio<T extends Unit> extends Ratio<T, T> {
    public SameTypeRatio(T from, T to) {
        super(from, to);
    }
}
