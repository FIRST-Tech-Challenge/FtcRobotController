package com.technototes.library.measurement.unit;

import java.util.function.DoubleSupplier;

public class RotationUnit extends Unit<RotationUnit.RotationUnitType> {

    public enum RotationUnitType {
        RADIANS(2 * Math.PI), DEGREES(360), ROTATIONS(1);
        public double relation;

        RotationUnitType(double d) {
            relation = d;
        }
    }
    public RotationUnit(DoubleSupplier v, RotationUnitType t){
        super(v, t);
    }

    @Override
    public double to(RotationUnitType type) {
        return get()*type.relation;
    }

    @Override
    public double get() {
        return (value.getAsDouble()/unitType.relation);
    }

}
