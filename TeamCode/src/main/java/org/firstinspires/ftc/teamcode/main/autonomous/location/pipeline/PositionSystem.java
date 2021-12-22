package org.firstinspires.ftc.teamcode.main.autonomous.location.pipeline;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardDistanceSensor;

public class PositionSystem {
    public Axis leftToRight;
    public Axis upAndDown;

    public CoordinateSystem coordinateSystem;

    public PositionSystem(AllSensors sensors) {
        leftToRight = new Axis(sensors.east, sensors.west);
        upAndDown = new Axis(sensors.north, sensors.north);

        coordinateSystem = new CoordinateSystem();
    }

    private void UpdateCoordinateSystem(CoordinateSystem.FieldCoordinates coordinates) {
        coordinateSystem.Update(coordinates);
    }

    public void SetAngle(double angle, AngleUnit unit) {
        switch (unit) {
            case DEGREES:
                coordinateSystem.angleDegrees = angle;
                break;
            case RADIANS:
                coordinateSystem.angleDegrees = Math.toDegrees(angle);
                break;
        }
    }

    public void GetAndEvalReadings() {
        Axis.AxisReading ew = leftToRight.getReadings();
        Axis.AxisReading ns = upAndDown.getReadings();

        EvalReadings(ew, ns);
    }

    private void EvalReadings(Axis.AxisReading eastWest, Axis.AxisReading northSouth) {
        boolean eastWestValid = true;
        boolean northSouthValid = true;

        // Check which axes are valid
        if (eastWest.sensor1 + eastWest.interSensorDistance + eastWest.sensor2 + 10 > CoordinateSystem.maxWidthInCM)
            eastWestValid = false;

        if (northSouth.sensor1 + northSouth.interSensorDistance + northSouth.sensor2 + 10 > CoordinateSystem.maxLengthInCM)
            northSouthValid = false;

        if (!northSouth.sensor1Valid & !northSouth.sensor2Valid) northSouthValid = false;
        if (!eastWest.sensor1Valid & !eastWest.sensor2Valid) eastWestValid = false;

        double x = eastWest.sensor1;
        double y = northSouth.sensor1;

        // do some geometry-I honors level math
        double angleDegrees = coordinateSystem.angleDegrees;
        double angleRadians = Math.toRadians(angleDegrees);

        x = x * Math.sin(angleRadians);
        y = y * Math.cos(angleRadians);

        if (x < 0 && eastWest.sensor2Valid) {
            x = eastWest.sensor2 * Math.sin(angleRadians);
        }
        else if (x < 0) {
            x = CoordinateSystem.maxWidthInCM - eastWest.interSensorDistance - Math.abs(x);
        }

        if (y < 0) {
            y = CoordinateSystem.maxLengthInCM - northSouth.interSensorDistance - Math.abs(y);
        }

        // Do our updating
        if (eastWestValid && northSouthValid) {
            coordinateSystem.Update(CoordinateSystem.FieldCoordinates.make(x, y));
        } else if (eastWestValid) {
            coordinateSystem.Update(CoordinateSystem.FieldCoordinates.make(x, coordinateSystem.current.y));
        } else if (northSouthValid) {
            coordinateSystem.Update(CoordinateSystem.FieldCoordinates.make(coordinateSystem.current.x, y));
        }
    }

    public static class AllSensors {
        public StandardDistanceSensor north;
        public StandardDistanceSensor east;
        public StandardDistanceSensor west;

        public AllSensors(StandardDistanceSensor north, StandardDistanceSensor east, StandardDistanceSensor west) {
            this.north = north;
            this.east = east;
            this.west = west;
        }
    }
}
