package com.mrcod.meepmeep;

public class MeepMeepHelper {
    /**
     * Field is 12 by 12, coordinates are 140 by 140, which means that we can multiply feet by
     * 140/12, which is equivalent to 35/3.
     * @param feet size in feet
     * @return size in coordinates
     */
    public static double feetToCoordinate(double feet) {
        return feet*35/3;
    }

    /**
     * Since changing feet to coords is multiplying by 3/35, and a foot is 12 inches, we can
     * multiply inches by 35/3 * 1/12, which is equivalent to 35/36.
     */
    public static double inchesToCoordinate(double inches) {
        return inches*35/36;
    }
}
