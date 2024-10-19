package org.firstinspires.ftc.teamcode.CompBot;


// Stuff that doesn't do anything physically, only math n' stuff goes here

public class Utils {


    /**
     * Converts standard cartesian coordinates to polar coordinates
     *
     * @param x X input
     * @param y Y input
     * @return Returns an array of two doubles,<br>
     * <p>
     * [0] = R - Magnitude<br>
     * [1] = Theta Angle of input coordinate.<br>
     * Relative to unit circle, where 0deg in is to the right, 90 is up and 180 is left.
     * @see #polarToCartesian(double, double)
     */
    public double[] cartesianToPolar(double x, double y) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // Radius
        arrayToReturn[1] = Math.atan(y / x) * (Math.PI / 180); // Theta


        return arrayToReturn;
    }


    /**
     * Converts polar coordinates to cartesian
     *
     * @param r     Magnitude of input coordinate
     * @param theta Angle of input coordinate.<br>
     *              Relative to unit circle, where 0deg in is to the right, 90 is up and 180 is left.
     * @return Returns an array of two doubles,<br>
     * <p>
     * [0] = X<br>
     * [1] = Y
     * @see #cartesianToPolar(double, double)
     */
    public double[] polarToCartesian(double r, double theta) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = r * Math.cos(theta); // X
        arrayToReturn[1] = r * Math.sin(theta); // Y

        return arrayToReturn;
    }

}
