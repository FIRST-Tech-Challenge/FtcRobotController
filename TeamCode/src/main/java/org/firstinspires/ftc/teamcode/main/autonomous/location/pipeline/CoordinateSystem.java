package org.firstinspires.ftc.teamcode.main.autonomous.location.pipeline;

public class CoordinateSystem {
    public static double maxWidthInCM = 381;
    public static double maxLengthInCM = 381;

    public FieldCoordinates current = FieldCoordinates.TOP_LEFT;
    public FieldCoordinates last = FieldCoordinates.TOP_LEFT;
    public double angleDegrees;

    public void Update(FieldCoordinates coordinates) {
        last = current;
        current = coordinates;
    }

    public enum FieldCoordinates {
        TOP_LEFT(0, 0),
        TOP_RIGHT(maxWidthInCM, 0),
        BOTTOM_LEFT(0, maxLengthInCM),
        BOTTOM_RIGHT(maxWidthInCM, maxLengthInCM);

        public double x;
        public double y;

        FieldCoordinates(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public FieldCoordinates set(double x, double y) {
            this.x = x;
            this.y = y;
            return this;
        }

        public static FieldCoordinates make(double x, double y) {
            FieldCoordinates temp = TOP_LEFT;
            temp.x = x;
            temp.y = y;
            return temp;
        }
    }
}
