package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.utils.Pose2D;

/** This class serves to refactor a parametric with an actual length
 * @author TheConverseEngineer
 */
public class Path {

    private Parametric parametric;
    public double length;

    /** Convenience overload with no weight */
    public Path(double startX, double startY, double startAngle, double endX, double endY, double endAngle) {
        this(startX, startY, startAngle, endX, endY, endAngle, Math.sqrt( Math.pow(startX - endX, 2) + Math.pow(startY - endY, 2)) / 2);
    }

    /** Convenience overload with one weight */
    public Path(double startX, double startY, double startAngle, double endX, double endY, double endAngle, double weight) {
        this(startX, startY, startAngle, weight, endX, endY, endAngle, weight);
    }

    /** Constructor for class Path
     * @param startX          The starting x position
     * @param startY          The starting y position
     * @param startAngle      The starting angle
     * @param startWeight     The starting derivative weight
     * @param endX            The ending x position
     * @param endY            The ending y position
     * @param endAngle        The ending angle
     * @param endWeight       The ending derivative weight
     */
    public Path (double startX, double startY, double startAngle, double startWeight, double endX, double endY, double endAngle, double endWeight) {
        parametric = new Parametric(startX, startY, startAngle, startWeight, endX, endY, endAngle, endWeight);
        length = parametric.approxLength();
    }

    public Pose2D getPoint(double displacement) {
        return new Pose2D(parametric.getPoint(displacement / this.length), parametric.getDeriv(displacement / this.length));
    }

    public double getHeadingVelocity(double displacement) {
        return (parametric.getDeriv(displacement / this.length) - parametric.getDeriv(displacement / this.length + 0.0001)) / -0.0001;
    }

}
