package com.technototes.library.measurement.unit;

import java.util.function.DoubleSupplier;

public class MotorEncoderUnit extends Unit<MotorEncoderUnit.MotorEncoderUnitType> {
    public enum MotorEncoderUnitType{
        //NEVEREST
        NEVEREST_60(1680), NEVEREST_40(1120), NEVEREST_20(537.6), NEVEREST_3_7(103.6), NEVEREST_1(28),
        //GOBILDA
        GOBILDA_188(5264), GOBILDA_139(3892), GOBILDA_99_5(2786), GOBILDA_71_2(1993.6), GOBILDA_50_9(1425.2),
        GOBILDA_26_9(753.2), GOBILDA_19_2(537.6), GOBILDA_13_7(383.6), GOBILDA_5_2(145.6), GOBILDA_3_7(103.6), GOBILDA_1(28),
        //NONE
        DEFAULT(1);
        public double tpr;
        MotorEncoderUnitType(double t){
            tpr = t;
        }
        public double getRevolutions(){
            return tpr;
        }
    }
    public MotorEncoderUnit(DoubleSupplier v, MotorEncoderUnitType e) {
        super(v, e);
    }
    public MotorEncoderUnit(MotorEncoderUnitType e) {
        super(() -> 0, e);
    }

    @Override
    public double to(MotorEncoderUnitType motorEncoderUnitType) {
        return motorEncoderUnitType.tpr*get();
    }

    public double to(RotationUnit.RotationUnitType t){
        return get()*t.relation;
    }

    @Override
    public double get() {
        return value.getAsDouble()/unitType.tpr;
    }

}
