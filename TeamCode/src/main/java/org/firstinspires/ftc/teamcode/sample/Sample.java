package org.firstinspires.ftc.teamcode.sample;

import org.locationtech.jts.algorithm.ConvexHull;
import org.locationtech.jts.geom.*;
import org.locationtech.jts.geom.util.AffineTransformation;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;
import java.util.List;

public class Sample {
    public static final float targetTx = 0.2f;
    public static final float targetTy = 0.2f;
    public static final float targetArea = 15.0f;
    public static final int redSamplePipeline = 1;
    public static final int blueSamplePipeline = 2;
    public static final int sharedSamplePipeline = 0;
    public LLResult llResult;
    public Polygon mbr;
    public LLResultTypes.ColorResult colorResult;

    // constructor
    public Sample(LLResult llResult) {
        this.llResult = llResult;
    }

    /**
     * Check if the LLResult is valid, return 1 if valid, negative if not (with different error codes
     *
     * @return
     */
    public int isLLResultValid() {

        if (llResult != null) {
            if (llResult.isValid()) {
                if (llResult.getColorResults().size() == 1) {
                    colorResult = llResult.getColorResults().get(0);
                    // DEBUG : System.out.println("Retangle : " + getMinimumBoundingRectangle());
                    if (colorResult.getTargetCorners().size() >= 4) {
                        // TODO : check W/H ratio ??
                        return 1;
                    } else {
                        return -20000 - colorResult.getTargetCorners().size();
                    }
                } else {
                    return -10000 - llResult.getColorResults().size();
                }
            } else {
                return -2;
            }
        } else {
            return -1;
        }
    }

    public int getPipelineIndex() {
        return llResult.getPipelineIndex();
    }

    public String getColor() {
        switch (getPipelineIndex()) {
            case sharedSamplePipeline:
                return "Shared";
            case redSamplePipeline:
                return "Red";
            case blueSamplePipeline:
                return "Blue";
            default:
                return "Unknown";
        }
    }
    public double getDeltaX() {
        return Math.round((llResult.getTx() - targetTx) * 100.0) / 100.0;
    }

    public double getDeltaY() {
        return Math.round((llResult.getTy() - targetTy) * 100.0) / 100.0;
    }

    /**
     * Calculate the distance to the target on XY plane
     *
     * @return
     */
    public double distanceXY() {
        return Math.round(Math.sqrt(Math.pow(getDeltaX(), 2) + Math.pow(getDeltaY(), 2)) * 100.0) / 100.0;
    }
    /**
     * Calculate the delta to the target distance based on the area (distance is proportional to the square root of the area)
     *
     * @return
     */
    public double getDistance() {
        return Math.round((Math.sqrt(targetArea) - Math.sqrt(llResult.getTa())) * 1000.0) / 1000.0;
    }

    /**
     * Calculate the minimum bounding rectangle around the target corners
     *
     * @return
     */
    public Polygon getMinimumBoundingRectangle() {
        if (mbr == null) {
            // create a polygon from colorResult.getTargetCorners()
            List<Coordinate> coordinates = new ArrayList<>();
            for (List<Double> corner : colorResult.getTargetCorners()) {
                coordinates.add(new Coordinate(corner.get(0), corner.get(1)));
            }
            coordinates.add(new Coordinate(colorResult.getTargetCorners().get(0).get(0), colorResult.getTargetCorners().get(0).get(1)));
            GeometryFactory geometryFactory = new GeometryFactory();
            Polygon polygon = geometryFactory.createPolygon(coordinates.toArray(new Coordinate[0]));

            // Compute the convex hull of the polygon
            ConvexHull convexHull = new ConvexHull(polygon);
            Geometry hull = convexHull.getConvexHull();

            // Initialize the minimum area and the corresponding rectangle
            double minArea = Double.MAX_VALUE;
            Polygon minRectangle = null;

            // Iterate over each edge of the convex hull
            for (int i = 0; i < hull.getNumPoints() - 1; i++) {
                Coordinate p1 = hull.getCoordinates()[i];
                Coordinate p2 = hull.getCoordinates()[i + 1];

                // Compute the angle of the edge with the x-axis
                double angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);

                // Rotate the convex hull to align the edge with the x-axis
                AffineTransformation rotation = AffineTransformation.rotationInstance(-angle, p1.x, p1.y);
                Geometry rotatedHull = rotation.transform(hull);

                // Compute the bounding box of the rotated convex hull
                Envelope envelope = rotatedHull.getEnvelopeInternal();
                double area = envelope.getWidth() * envelope.getHeight();

                // Update the minimum area and the corresponding rectangle
                if (area < minArea) {
                    minArea = area;
                    Coordinate[] rectangleCoords = new Coordinate[]{
                            new Coordinate(envelope.getMinX(), envelope.getMinY()),
                            new Coordinate(envelope.getMaxX(), envelope.getMinY()),
                            new Coordinate(envelope.getMaxX(), envelope.getMaxY()),
                            new Coordinate(envelope.getMinX(), envelope.getMaxY()),
                            new Coordinate(envelope.getMinX(), envelope.getMinY())
                    };
                    minRectangle = polygon.getFactory().createPolygon(rectangleCoords);
                    // rotate the rectangle back
                    AffineTransformation reverseRotation = AffineTransformation.rotationInstance(angle, p1.x, p1.y);
                    mbr = (Polygon) reverseRotation.transform(minRectangle);
                }
            }
        }
        return mbr;
    }
    public void printPolygonCorners(Polygon polygon) {
        for (Coordinate coordinate : polygon.getCoordinates()) {
            System.out.println("Corner: (" + coordinate.x + ", " + coordinate.y + ")");
        }
    }
    /**
     * Calculate the angle (in degree) of the sample based on the target corners
     *    Up direction is 0 degree
     *    Left direction is 90 degree
     *    Right direction is -90 degree
     *
     * @return
     */
    public int getSampleAngle() {
        getMinimumBoundingRectangle();
        Coordinate p1 = mbr.getCoordinates()[0];
        Coordinate p2 = mbr.getCoordinates()[1];
        Coordinate p4 = mbr.getCoordinates()[3];
        //get polygon edges length
        double edgeP12Length = Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
        double edgeP14Length = Math.sqrt(Math.pow(p1.getX() - p4.getX(), 2) + Math.pow(p1.getY() - p4.getY(), 2));
        int raw_angle;
        // get angle of longer edge
        if (edgeP12Length > edgeP14Length) {
            raw_angle = -90-(int) Math.toDegrees(Math.atan2(p1.getY() - p2.getY(), p1.getX() - p2.getX()));
        } else {
            raw_angle = -90-(int) Math.toDegrees(Math.atan2(p1.getY() - p4.getY(), p1.getX() - p4.getX()));
        }
        // normalized to -90 ~ +90 degrees
        if (raw_angle > 90){
            return raw_angle - 180;
        }
        else if (raw_angle < -90){
            return raw_angle + 180;
        }
        return raw_angle;
    }

    /**
     * Return wide/height ratio of the sample
     * @return
     */
    public double getSampleWHRatio(){
        getMinimumBoundingRectangle();
        Coordinate p1 = mbr.getCoordinates()[0];
        Coordinate p2 = mbr.getCoordinates()[1];
        Coordinate p4 = mbr.getCoordinates()[3];
        //get polygon edges length
        double edgeP12Length = Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
        double edgeP14Length = Math.sqrt(Math.pow(p1.getX() - p4.getX(), 2) + Math.pow(p1.getY() - p4.getY(), 2));
        return edgeP12Length > edgeP14Length ?
               Math.round(edgeP12Length / edgeP14Length * 100.0)/100.0 :
               Math.round(edgeP14Length / edgeP12Length * 100.0)/100.0 ;
    }
    public String toString() {
        int result = isLLResultValid();
        if (result > 0) {
            return "Sample{" +
                    "x=" + getDeltaX() +
                    ", y=" + getDeltaY() +
                    ", d=" + getDistance() +
                    ", a=" + getSampleAngle() +
                    ", whr=" + getSampleWHRatio() +
                    ", c=" + getColor() +
                    '}';
        } else {
            return "Invalid LLResult with error code " + result;
        }
    }
}
