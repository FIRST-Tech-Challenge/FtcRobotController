package com.technototes.library.measurement.ratio;

import com.technototes.library.measurement.unit.DistanceUnit;
import com.technototes.library.measurement.unit.RotationUnit;

public class RotationToDistanceRatio extends Ratio<RotationUnit, DistanceUnit> {
    public RotationToDistanceRatio(RotationUnit from, DistanceUnit to) {
        super(from, to);
    }
}
