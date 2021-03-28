package com.technototes.library.measurement.unit;
@Deprecated
public class DistanceUnit extends Unit<DistanceUnit.DistanceUnitType> {

    public enum DistanceUnitType {
        CENTIMETERS(100), METERS(1), INCHES(39.37008), FEET(3.28084);
        public double relation;

        DistanceUnitType(double d) {
            relation = d;
        }
    }
    public DistanceUnit(double v, DistanceUnitType t){
        super(v, t);
    }

    @Override
    public double to(DistanceUnitType type) {
        return get()*type.relation;
    }

    @Override
    public double get() {
        return (value/((DistanceUnitType)unitType).relation);
    }

}
