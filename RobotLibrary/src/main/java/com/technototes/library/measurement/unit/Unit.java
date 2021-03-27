package com.technototes.library.measurement.unit;
@Deprecated
public abstract class Unit<D extends Enum<D>> {
    public D unitType;
    public double value;
    public Unit(double v, D e){
        unitType = e;
        value = v;
    }
    public abstract double to(D d);
    public double to(Unit<D> d){
        return to(d.unitType);
    }
    public abstract double get();


}
