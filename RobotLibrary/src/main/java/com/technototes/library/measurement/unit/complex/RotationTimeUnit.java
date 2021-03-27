package com.technototes.library.measurement.unit.complex;

import com.technototes.library.measurement.unit.RotationUnit;
import com.technototes.library.measurement.unit.TimeUnit;
@Deprecated
public class RotationTimeUnit extends ComplexUnit<RotationUnit.RotationUnitType, TimeUnit.TimeUnitType> {

    public RotationTimeUnit(double val, RotationUnit.RotationUnitType e1, TimeUnit.TimeUnitType e2) {
        super(val, e1, e2);
    }

    @Override
    public double to(Enum anEnum) {
        return 0;
    }

    @Override
    public double get() {
        return 0;
    }
}