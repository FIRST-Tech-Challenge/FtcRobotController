package org.firstinspires.ftc.teamcode.main.autonomous.location.pipeline;

import org.firstinspires.ftc.teamcode.competition.utils.sensors.SensorWrapper;
import org.firstinspires.ftc.teamcode.main.autonomous.location.CoordinateSystem;

import java.util.ArrayList;
import java.util.Dictionary;

public class PositionTracker {
    public Axis leftToRight;
    public Axis upAndDown;

    public CoordinateSystem coordinateSystem;

    public PositionTracker(AllSensors sensors) {
        leftToRight = new Axis(sensors.east, sensors.west);
        upAndDown = new Axis(sensors.north, sensors.west);

        coordinateSystem = new CoordinateSystem();
    }

    private void UpdateCoordinateSystem(CoordinateSystem.FieldCoordinates coordinates) {
        coordinateSystem.Update(coordinates);
    }

    private void GetAndEvalReadings() {
        Axis.AxisReading ew = leftToRight.getReadings();
        Axis.AxisReading ns = upAndDown.getReadings();

        EvalReadings(ew, ns);
    }

    private void EvalReadings(Axis.AxisReading eastWest, Axis.AxisReading northSouth) {
        // TODO: Implement EvalReadings

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

        if (x < 0) {
            x = eastWest.sensor2 * Math.sin(angleRadians);
        }
        if (y < 0) {

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
        public SensorWrapper north;
        public SensorWrapper south;
        public SensorWrapper east;
        public SensorWrapper west;

        public AllSensors(SensorWrapper north, SensorWrapper south, SensorWrapper east, SensorWrapper west) {
            this.north = north;
            this.south = south;
            this.east = east;
            this.west = west;
        }

        // No South-facing
        public AllSensors(SensorWrapper north, SensorWrapper east, SensorWrapper west) {
            this.north = north;
            this.south = null;
            this.east = east;
            this.west = west;
        }
    }
}
