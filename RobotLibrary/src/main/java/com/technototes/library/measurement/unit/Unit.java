package com.technototes.library.measurement.unit;

import java.util.function.DoubleSupplier;

public abstract class Unit<D extends Enum<D>> {
    public D unitType;
    public DoubleSupplier value;
    public Unit(DoubleSupplier v, D e){
        unitType = e;
        value = v;
    }
    public Unit<D> setSupplier(DoubleSupplier d){
        value = d;
        return this;
    }
    public abstract double to(D d);
    public double to(Unit<D> d){
        return to(d.unitType);
    }
    public abstract double get();


}
