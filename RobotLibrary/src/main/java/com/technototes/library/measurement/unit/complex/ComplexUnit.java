package com.technototes.library.measurement.unit.complex;

import com.technototes.library.measurement.unit.Unit;
@Deprecated
public abstract class ComplexUnit<T extends Enum<T>, U extends Enum<U>> extends Unit {
    public Enum<U> unitType2;
    public ComplexUnit(double val, T e1, U e2) {
        super(val, e1);
        unitType2 = e2;
    }


}
