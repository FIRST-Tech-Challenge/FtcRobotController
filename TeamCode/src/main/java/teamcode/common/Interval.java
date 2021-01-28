package teamcode.common;

public class Interval {

    private double min;
    boolean minInclusive;
    private double max;
    boolean maxInclusive;

    public Interval(double min, boolean minInclusive, double max, boolean maxInclusive) {
        this.min = min;
        this.minInclusive = minInclusive;
        this.max = max;
        this.maxInclusive = maxInclusive;
    }

    /**
     * Includes the min and max;
     */
    public Interval(double min, double max) {
        this(min, true, max, true);
    }

    public double getMin() {
        return min;
    }

    public void setMin(double min) {
        validateMinMax(min, this.max);
        this.min = min;
    }

    public boolean isMinInclusive() {
        return minInclusive;
    }

    public void setMinInclusive(boolean inclusive) {
        this.minInclusive = inclusive;
    }

    public double getMax() {
        return max;
    }

    public void setMax(double max) {
        validateMinMax(this.min, max);
        this.max = max;
    }

    public boolean isMaxInclusive() {
        return maxInclusive;
    }

    public void setMaxInclusive(boolean inclusive) {
        this.maxInclusive = inclusive;
    }

    public boolean contains(double n) {
        if (minInclusive) {
            if (n < min) {
                return false;
            }
        } else {
            if (n <= min) {
                return false;
            }
        }
        if (maxInclusive) {
            if (n > max) {
                return false;
            }
        } else {
            if (n >= max) {
                return false;
            }
        }
        return true;
    }

    private static void validateMinMax(double min, double max) {
        if (min > max) {
            throw new IllegalArgumentException("min > max/");
        }
    }

}
