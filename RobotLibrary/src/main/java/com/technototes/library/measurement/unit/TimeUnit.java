package com.technototes.library.measurement.unit;

import java.util.function.DoubleSupplier;

public class TimeUnit extends Unit<TimeUnit.TimeUnitType> {

    public enum TimeUnitType {
        SECONDS(1), MILLISECONDS(1000), MINUTES(60), HOUR(3600);
        public double relation;

        TimeUnitType(double d) {
            relation = d;
        }
    }
    public TimeUnit(DoubleSupplier v, TimeUnitType t){
        super(v, t);
    }

    @Override
    public double to(TimeUnitType type) {
        return get()*type.relation;
    }

    @Override
    public double get() {
        return (value.getAsDouble()/((TimeUnitType)unitType).relation);
    }

}
