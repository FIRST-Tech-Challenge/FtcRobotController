package com.technototes.library.measurement.ratio;

import com.technototes.library.measurement.unit.DistanceUnit;
import com.technototes.library.measurement.unit.RotationUnit;
@Deprecated
public class DistanceeToRotationRatio extends Ratio<DistanceUnit, RotationUnit> {
    public DistanceeToRotationRatio(DistanceUnit from, RotationUnit to) {
        super(from, to);
    }
}
