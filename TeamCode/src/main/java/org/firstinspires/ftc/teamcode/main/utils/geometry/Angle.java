package org.firstinspires.ftc.teamcode.main.utils.geometry;

public class Angle extends Number {

    public double value;
    public AngleUnit unit;

    public Angle(double value, AngleUnit unit) {
        this.value = value;
        this.unit = unit;
    }

    public double convert(AngleUnit newUnit) {
        if (newUnit != unit) {
            switch (newUnit) {
                case RADIAN:
                    value = Math.toDegrees(value);
                    break;
                case DEGREE:
                    value = Math.toRadians(value);
                    break;
            }
        }

        return value;
    }

    public int asDegree() {
        int temp = (int) value;
        if (unit != AngleUnit.DEGREE) { temp = (int) Math.toDegrees(value); }

        return temp;
    }
    public double asRadian() {
        double temp = value;
        if (unit != AngleUnit.RADIAN) { temp = Math.toRadians(value); }

        return temp;
    }

    public void normalize() {
        switch (unit) {
            case DEGREE:
                value = normalizeDegrees((int) value);
                break;
            case RADIAN:
                value = normalizeRadians(value);
                break;
        }
    }

    public static int normalizeDegrees(int value) {
        if (value < 0) {
            value = Math.abs(value) + 180;
        }
        return value;
    }
    public static double normalizeRadians(double value) {
        if (value < 0) {
            value = Math.abs(value) + Math.PI;
        }
        return value;
    }

    public boolean lessThan(Angle toCompare) {
        if (unit == toCompare.unit) {
            return value < toCompare.value;
        } else if (unit == AngleUnit.DEGREE) {
            return value < toCompare.asDegree();
        } else {
            return value < toCompare.asRadian();
        }
    }
    public boolean greaterThan(Angle toCompare) {
        if (unit == toCompare.unit) {
            return value > toCompare.value;
        } else if (unit == AngleUnit.DEGREE) {
            return value > toCompare.asDegree();
        } else {
            return value > toCompare.asRadian();
        }
    }
    public boolean equalTo(Angle toCompare) {
        if (unit == toCompare.unit) {
            return value == toCompare.value;
        } else if (unit == AngleUnit.DEGREE) {
            return value == toCompare.asDegree();
        } else {
            return value == toCompare.asRadian();
        }
    }

    @Override
    public int intValue() {
        return (int) value;
    }
    @Override
    public long longValue() {
        return (long) value;
    }
    @Override
    public float floatValue() {
        return (float) value;
    }
    @Override
    public double doubleValue() {
        return value;
    }

    @Override
    public String toString() {
        return value + unit.toString();
    }

    public enum AngleUnit {
        DEGREE("Degrees"), RADIAN("Radians");

        public String name;

        AngleUnit(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }
    }
}
