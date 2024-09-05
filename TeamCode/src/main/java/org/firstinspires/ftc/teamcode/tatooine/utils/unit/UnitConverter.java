package org.firstinspires.ftc.teamcode.tatooine.utils.unit;

public class UnitConverter {
    public static double convert(double value, unit from, unit to) {
        if (from == to) {
            return value;
        }
        switch (from) {
            case INCHES:
                switch (to) {
                    case CM:
                        return value * 2.54;
                    case MM:
                        return value * 25.4;
                    case FEET:
                        return value / 12;
                    case METERS:
                        return value * 0.0254;
                }
                break;
            case CM:
                switch (to) {
                    case INCHES:
                        return value / 2.54;
                    case MM:
                        return value * 10;
                    case FEET:
                        return value / 30.48;
                    case METERS:
                        return value * 0.01;
                }
                break;
            case MM:
                switch (to) {
                    case INCHES:
                        return value / 25.4;
                    case CM:
                        return value / 10;
                    case FEET:
                        return value / 304.8;
                    case METERS:
                        return value * 0.001;
                }
                break;
            case FEET:
                switch (to) {
                    case INCHES:
                        return value * 12;
                    case CM:
                        return value * 30.48;
                    case MM:
                        return value * 304.8;
                    case METERS:
                        return value * 0.3048;
                }
                break;
            case METERS:
                switch (to) {
                    case INCHES:
                        return value / 0.0254;
                    case CM:
                        return value / 0.01;
                    case MM:
                        return value / 0.001;
                    case FEET:
                        return value / 0.3048;
                }
                break;
        }
        return 0;
    }
}



