package org.firstinspires.ftc.teamcode;

//this class allows you to avoid dealing with angle conversions!
//different angle types are input into program and required for calculations (ex. IMU reading, vector math...)

// make sure you...
// initialize all Angle objects with the correct AngleType for the data you're storing
// use angle1.getDifference(angle2) to get the difference between two angles (DON'T subtract raw values)
//   ^and don't forget to use getDirection() to get the sign of the difference

//other notes:
// you do NOT need to convert angle types before getting the difference or direction between them
// you CAN initialize Angle objects with an angle value greater than the range (ex. new Angle(600, ZERO_TO_360_HEADING) is valid


//definitions:

//CW = clockwise, CCW = counter-clockwise

//Heading style: zero is forward (positive y-axis), CW is positive, CCW is negative
//Cartesian style: zero is right (positive x-axis), CCW is positive, CW is negative

//ZERO_TO_360_HEADING: heading style, going around CW from 0 takes you up to 360 and back to 0
//NEG_180_TO_180_HEADING: heading style, going CW from 0 takes you up to +180, going CCW takes you to -180
//ZERO_TO_360_CARTESIAN: cartesian style, going CCW from 0 takes you up to 360 and back to 0
//NEG_180_TO_180_CARTESIAN: cartesian style, going CCW from 0 takes you up to +180, going CW takes you to -180


public class Angle {

    //relative to robot starting position (right = east, left = west, forward = north, backward = south)
    public static final Angle
            RIGHT = new Angle(90, AngleType.NEG_180_TO_180_HEADING),
            LEFT = new Angle(-90, AngleType.NEG_180_TO_180_HEADING),
            BACKWARD = new Angle(180, AngleType.NEG_180_TO_180_HEADING),
            FORWARD = new Angle(0, AngleType.NEG_180_TO_180_HEADING);


    //see top for type definitions
    enum AngleType {
        ZERO_TO_360_CARTESIAN, ZERO_TO_360_HEADING, NEG_180_TO_180_CARTESIAN, NEG_180_TO_180_HEADING
    }

    //not stored for each angle, but used to return direction between two angles
    enum Direction {
        CLOCKWISE, COUNTER_CLOCKWISE
    }

    private double angle;
    private final AngleType type;

    public Angle (double angle, AngleType type) {
        this.angle = angle;
        this.type = type;

        //handles case of input angle outside of range (ex. angle = 600)
        this.angle = convertAngleDouble(type);
    }

    public double getAngle (AngleType type) {
        return this.convertAngle(type).getAngle();
    }

    public AngleType getType () { return type; }

    @Override
    public String toString () {
        return "" + angle + " :" + type;
    }

    //assumes DEGREES for input and output!! use built-in Java method to convert between radians and degrees
    //no other assumptions related to inputAngle value (can be -infinity to infinity)
    public Angle convertAngle (AngleType outputType) {
        return new Angle (convertAngleDouble(outputType), outputType);
    }

    public double convertAngleDouble (AngleType outputType) {
        //handles case of same input and output type
        if (type == outputType) {
            return wrapAngle(this.getAngle(), outputType); //was new Angle(angle, type)
        }

        if (sameNumericalSystem(type, outputType)) {
            return convertCoordinateSystem(angle, type, outputType);
        }
        else if (sameCoordinateSystem(type, outputType)) {
            return convertNumericalSystem(angle, type, outputType);
        }
        else {
            //even though input and output types are not true to the type of intermediate angle...
            // they have the correct important characteristic (numerical or coordinate)
            double angleNewNumericalSystem = convertNumericalSystem(angle, type, numericalAndCoordinate(outputType, type)); //was type, output type
            double angleNewCoordinateSystem = convertCoordinateSystem(angleNewNumericalSystem, numericalAndCoordinate(outputType, type), outputType); //was type, output type
            return angleNewCoordinateSystem;
        }
    }

    //returns absolute value of difference between two Angles (can be any type)
    //min return value is 0 and max return value is 180
    public double getDifference (Angle other) {
        Angle otherConverted = other.convertAngle(AngleType.ZERO_TO_360_CARTESIAN);
        Angle thisConverted = this.convertAngle(AngleType.ZERO_TO_360_CARTESIAN);

        double rawDiff = Math.abs(otherConverted.getAngle() - thisConverted.getAngle());
        if (rawDiff > 180) {
            return 360 - rawDiff; //will be positive bc 360 is max rawDiff
        }
        return rawDiff; //number between  0 and 180
    }

    //returns direction of travel FROM this angle TO other angle
    //example: direction FROM 0 degrees TO 90 degrees (both in NEG_180_TO_180_HEADING type) is CLOCKWISE
    //returns either CLOCKWISE or COUNTER_CLOCKWISE
    //defaults to CLOCKWISE if angles are identical (difference is zero)
    public Direction directionTo (Angle other) {
        Angle otherConverted = other.convertAngle(AngleType.ZERO_TO_360_CARTESIAN);
        Angle thisConverted = this.convertAngle(AngleType.ZERO_TO_360_CARTESIAN);

        double rawDiff = Math.abs(otherConverted.getAngle() - thisConverted.getAngle());
        if (rawDiff > 180) {
            if (otherConverted.getAngle() > thisConverted.getAngle()) {
                return Direction.CLOCKWISE;
            } else {
                return Direction.COUNTER_CLOCKWISE;
            }
        } else {
            if (otherConverted.getAngle() > thisConverted.getAngle()) {
                return Direction.COUNTER_CLOCKWISE;
            } else {
                return Direction.CLOCKWISE;
            }
        }
    }

    //passing a negative degrees will work, but will reverse the direction
    //direction should indicate positive direction of the angle system being used
    public Angle rotateBy (double degrees, Direction direction) {
        Angle thisConverted = this.convertAngle(AngleType.ZERO_TO_360_HEADING);
        double newAngle;
        if (direction == Direction.CLOCKWISE) {
            newAngle = thisConverted.getAngle() + degrees;
        } else {
            newAngle = thisConverted.getAngle() - degrees;
        }
        return new Angle(newAngle, AngleType.ZERO_TO_360_HEADING).convertAngle(this.type);
    }

    //defaults to positive direction of this angle
    public Angle rotateBy (double degrees) {
        return rotateBy(degrees, this.getPositiveDirection());
    }

    public static Angle getAverageAngle (Angle angle1, Angle angle2) {
        double difference = angle1.getDifference(angle2);
        Direction direction = angle1.directionTo(angle2);
        return angle1.rotateBy(difference/2.0, direction);
    }

    //INTERNAL METHODS - don't worry about these unless you're interested in how this class works

    //input and output type should have the same numerical system
    public static double convertCoordinateSystem (double inputAngle, AngleType inputType, AngleType outputType) {
        //ensure input and output coordinate system not same- assumed different later on (bc of *-1)
        if (sameCoordinateSystem(inputType, outputType)) {
            return inputAngle; //not sure about this
        }

        if (isCartesian(inputType)) {
            //+90 is to convert coordinate systems
            //wrapAngle is to make sure within bounds of numerical system
            //*-1 or 360- is to flip direction (coordinate system change always causes positive to flip between CW and CCW)
            if (isZeroTo360(inputType)) {
                return 360 - wrapAngle(inputAngle - 90, outputType); //flipped plus to minus (correct with minus)
            }
            else {
                return -1 * wrapAngle(inputAngle - 90, outputType);
            }
        } else { //input type is heading system
            if (isZeroTo360(inputType)) {
                return 360 - wrapAngle(inputAngle - 90, outputType); //WAS +90
            }
            else {
                return -1 * wrapAngle(inputAngle - 90, outputType); //WAS +90
            }
        }
    }

    //although this method currently is just a pass through, I think it may need to do more in the future (and it adds uniformity)
    public static double convertNumericalSystem (double inputAngle, AngleType inputType, AngleType outputType) {
        if (sameNumericalSystem(inputType, outputType)) {
            return inputAngle; //for uniformity
        }
        return wrapAngle(inputAngle, outputType);
    }

    public static boolean sameCoordinateSystem(AngleType firstType, AngleType secondType) {
        return isCartesian(firstType) == isCartesian(secondType);
    }

    public static boolean sameNumericalSystem(AngleType firstType, AngleType secondType) {
        return isZeroTo360(firstType) == isZeroTo360(secondType);
    }

    public static boolean isCartesian (AngleType angleType) {
        if (angleType == AngleType.ZERO_TO_360_CARTESIAN || angleType == AngleType.NEG_180_TO_180_CARTESIAN) {
            return true;
        }
        return false;
    }

    public static boolean isZeroTo360 (AngleType angleType) {
        if (angleType == AngleType.ZERO_TO_360_CARTESIAN || angleType == AngleType.ZERO_TO_360_HEADING) {
            return true;
        }
        return false;
    }

    public static AngleType numericalAndCoordinate (AngleType numericalType, AngleType coordinateType) {
        if (isZeroTo360(numericalType) && isCartesian(coordinateType)) return AngleType.ZERO_TO_360_CARTESIAN;
        else if (!isZeroTo360(numericalType) && isCartesian(coordinateType)) return AngleType.NEG_180_TO_180_CARTESIAN;
        else if (isZeroTo360(numericalType) && !isCartesian(coordinateType)) return AngleType.ZERO_TO_360_HEADING;
        else return AngleType.NEG_180_TO_180_HEADING; //!isZeroTo360(numericalType) && !isCartesian(coordinateType)
    }

    public Direction getPositiveDirection () {
        if (this.type == AngleType.NEG_180_TO_180_HEADING || this.type == AngleType.ZERO_TO_360_HEADING) {
            return Direction.CLOCKWISE;
        }
        return Direction.COUNTER_CLOCKWISE;
    }

    //returns an angle between max and min, assuming a coordinate system starting at min and wrapping back to max
    //assumes min < max AND min <= 0
    public static double wrapAngle(double angle, double min, double max) {
        angle = mod(angle, range(min, max));
        if (angle > max) { //won't be < min bc of second assumption
            return min + min + angle; //I have no idea why, but it seems to work for all cases under assumptions (?)
        }
        return angle;
    }

    //shortcut for AngleType instead of min and max bounds
    public static double wrapAngle(double angle, AngleType outputAngleType) {
        if (isZeroTo360(outputAngleType)) {
            return wrapAngle(angle, 0, 360);
        } else {
            return wrapAngle(angle, -180, 180);
        }
    }

    //returns the range between two numbers (ex. -180, 180 returns 360)
    public static double range (double num1, double num2) {
        return Math.abs(num1-num2);
    }

    //returns python version of n % m (n is dividend, m is divisor)
    //python % never returns negative numbers, but Java % does
    public static double mod (double n, double m) {
        return (((n % m) + m) % m);
    }
}