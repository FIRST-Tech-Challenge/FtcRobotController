package com.technototes.library.measurement.unit;
@Deprecated
public class RotationUnit extends Unit<RotationUnit.RotationUnitType> {

    public enum RotationUnitType {
        RADIANS(2 * Math.PI), DEGREES(360), ROTATIONS(1);
        public double relation;

        RotationUnitType(double d) {
            relation = d;
        }
    }
    public RotationUnit(double v, RotationUnitType t){
        super(v, t);
    }

    @Override
    public double to(RotationUnitType type) {
        return get()*type.relation;
    }

    @Override
    public double get() {
        return (value/((RotationUnitType)unitType).relation);
    }

}
