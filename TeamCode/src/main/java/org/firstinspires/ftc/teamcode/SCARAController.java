package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class SCARAController {
    double arm1Length, arm2Length;

    double theta1; // angle of arm1 relative to the frame
    double theta2; // angle of arm2 relative to arm1

    ClawPosition clawInsideRobot;
    ClawPosition clawUnderBridge;
    ClawPosition clawOutsideRobot;
    ClawPosition clawAtDelivery;

    ClawPosition lastPosition;

    double servo1TicksPerRadian, servo2TicksPerRadian;

    final static double ARM_SPEED = 350; // mm per second

    final static double SIDE_TO_SIDE_RANGE = 8 * 25.4;
    final static double MIDLINE = -76;
    final static double CALIBRATION_Y_DISTANCE = 200;

    //shift delivery and pickup forward by 25 cm to account for the new claw attach point
    final static double DELIVER_Y_DISTANCE = 6.5 * 25.4 - 25;

    final static double PICK_UP_Y_DISTANCE = -180; // had -25 (negative closer to front of robot)
    //    final static double ARM1_SERVO_POSITION_90_DEGREES = .801;
//    final static double ARM1_SERVO_POSITION_270_DEGREES = .159;
//    final static double ARM2_SERVO_POSITION_90_DEGRESS = .392;
//    final static double ARM2_SERVO_POSITION_180_DEGREES = .764;
    final static double ARM1_SERVO_POSITION_90_DEGREES = .807; //.787;
    final static double ARM1_SERVO_POSITION_270_DEGREES = .144; //.121;
    final static double ARM2_SERVO_POSITION_90_DEGRESS = .163;//.134; //.171;
    final static double ARM2_SERVO_POSITION_180_DEGREES = .529;//.580;//.535;

    Sequence INSIDE_ROBOT_TO_DELIVERY;
    Sequence DELIVERY_TO_INSIDE_ROBOT;

    Telemetry telemetry;
    public SCARAController(double arm1Length, double arm2Length, Telemetry telemetry) {
        this.arm1Length = arm1Length;
        this.arm2Length = arm2Length;
        this.telemetry = telemetry;

        ArmAngles armAngles = new ArmAngles(0,0);
//        clawInsideRobot = new ClawPosition(MIDLINE, -CALIBRATION_Y_DISTANCE, .119, .883);
        clawInsideRobot = new ClawPosition(MIDLINE, PICK_UP_Y_DISTANCE, .119, .883);

        // need to update angles for inside the robot.  This is the reset position
        clawInsideRobot.armAngles.setAngles(clawInsideRobot.coordinates, true);
        clawInsideRobot.servoPositions.updateFromControlAngles(clawInsideRobot.armAngles);


        clawUnderBridge = new ClawPosition(MIDLINE, 0, .202, .411);
//        clawOutsideRobot = new ClawPosition(MIDLINE, OUTSIDE_DISTANCE, .801, .346);
        clawOutsideRobot = new ClawPosition(MIDLINE, CALIBRATION_Y_DISTANCE, .690, .375);
        clawAtDelivery = new ClawPosition(MIDLINE, DELIVER_Y_DISTANCE, .520, .357);

//        servo1TicksPerRadian = ((clawOutsideRobot.servoPositions.servo1 - clawInsideRobot.servoPositions.servo1) / -getAngleDifferenceCounterClockwise(clawOutsideRobot.armAngles.angle1, clawInsideRobot.armAngles.angle1));
//        servo2TicksPerRadian = (clawInsideRobot.servoPositions.servo2 - clawUnderBridge.servoPositions.servo2)
////                / getAngleDifferenceClockwise(getAngleDifferenceClockwise(clawInsideRobot.theta1, clawInsideRobot.theta2), getAngleDifferenceClockwise(clawUnderBridge.theta1, clawUnderBridge.theta2)));
//                / getAngleDifferenceCounterClockwise(clawUnderBridge.armAngles.angle1 + clawUnderBridge.armAngles.angle2, clawInsideRobot.armAngles.angle1 + clawInsideRobot.armAngles.angle2);

        servo1TicksPerRadian = getServo1TicksPerRadian(clawInsideRobot, clawOutsideRobot);
        servo2TicksPerRadian = getServo2TicksPerRadian(clawInsideRobot, clawUnderBridge);
//        servo1TicksPerRadian = getServo1TicksPerRadian(clawInsideRobot, clawAtDelivery);
//        servo2TicksPerRadian = getServo2TicksPerRadian(clawInsideRobot, clawAtDelivery);

        lastPosition = clawInsideRobot;
        INSIDE_ROBOT_TO_DELIVERY = new Sequence(new Coordinates(clawInsideRobot.coordinates.x, clawInsideRobot.coordinates.y), new Coordinates(MIDLINE, DELIVER_Y_DISTANCE));
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(0).adjust(0, Math.PI * 0.0 / 180);
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(1).adjust(0, Math.PI * -30.0 / 180);
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(2).adjust(0, Math.PI * -40.0 / 180);
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(3).adjust(0, Math.PI * -40.0 / 180);
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(4).adjust(0, Math.PI * -10.0 / 180);
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(5).adjust(0, Math.PI * 0.0 / 180);
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(6).adjust(0, Math.PI * -10.0 / 180);
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(7).adjust(0, Math.PI * 0.0 / 180);
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(8).adjust(0, Math.PI * 10.0 / 180);
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(9).adjust(0, Math.PI * 10.0 / 180);
//        INSIDE_ROBOT_TO_DELIVERY.angleList.get(10).adjust(0, Math.PI * 10.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(0).adjust(Math.PI * 0.0 / 180, Math.PI * 0.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(1).adjust(Math.PI * 0.0 / 180, Math.PI * -10.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(2).adjust(Math.PI * 0.0 / 180, Math.PI * 0.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(3).adjust(Math.PI * 0.0 / 180, Math.PI * 0.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(4).adjust(Math.PI * 10.0 / 180, Math.PI * 0.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(5).adjust(Math.PI * 20.0 / 180, Math.PI * -20.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(6).adjust(Math.PI * 30.0 / 180, Math.PI * -30.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(7).adjust(Math.PI * 20.0 / 180, Math.PI * -20.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(8).adjust(Math.PI * 10.0 / 180, Math.PI * 20.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(9).adjust(Math.PI * 0.0 / 180, Math.PI * 10.0 / 180);
        INSIDE_ROBOT_TO_DELIVERY.angleList.get(10).adjust(Math.PI * 0.0 / 180, Math.PI * 10.0 / 180);

        DELIVERY_TO_INSIDE_ROBOT = new Sequence(new Coordinates(MIDLINE, DELIVER_Y_DISTANCE), new Coordinates(clawInsideRobot.coordinates.x, clawInsideRobot.coordinates.y));
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(0).adjust(Math.PI * 0.0 / 180, Math.PI * 0.0 / 180);
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(1).adjust(Math.PI * 0.0 / 180, Math.PI * 0.0 / 180);
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(2).adjust(Math.PI * 0.0 / 180, Math.PI * -10.0 / 180);
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(3).adjust(Math.PI * 0.0 / 180, Math.PI * -20.0 / 180);
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(4).adjust(Math.PI * 0.0 / 180, Math.PI * -20.0 / 180);
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(5).adjust(Math.PI * -10.0 / 180, Math.PI * 20.0 / 180);
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(6).adjust(Math.PI * -20.0 / 180, Math.PI * 20.0 / 180);
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(7).adjust(Math.PI * -20.0 / 180, Math.PI * 10.0 / 180);
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(8).adjust(Math.PI * 0.0 / 180, Math.PI * 0.0 / 180);
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(9).adjust(Math.PI * 0.0 / 180, Math.PI * 0.0 / 180);
        DELIVERY_TO_INSIDE_ROBOT.angleList.get(10).adjust(Math.PI * 0.0 / 180, Math.PI * 0.0 / 180);



    }


    public double getServo1TicksPerRadian(ClawPosition p1, ClawPosition p2) {
        return ((p2.servoPositions.servo1 - p1.servoPositions.servo1) / -getAngleDifferenceCounterClockwise(p2.armAngles.angle1, p1.armAngles.angle1));
    }

    public double getServo2TicksPerRadian(ClawPosition p1, ClawPosition p2) {
        return (p1.servoPositions.servo2 - p2.servoPositions.servo2) / getAngleDifferenceCounterClockwise(p2.armAngles.angle1 + p2.armAngles.angle2, p1.armAngles.angle1 + p1.armAngles.angle2);

    }

//    public boolean calculateAngles(double x, double y, boolean inverted, ArmAngles armAngles) {
//        double cos_theta2 = (x*x + y*y - arm1Length * arm1Length - arm2Length * arm2Length) / (2 * arm1Length * arm2Length);
//
//        // first term can be plus or minus resulting in two different solutions for the same position
//        double newTheta2 = Math.atan2((inverted ? -1 : 1) * Math.sqrt(1 - cos_theta2 * cos_theta2), cos_theta2);
//        double k1 = arm1Length + arm2Length * Math.cos(newTheta2);
//        double k2 = arm2Length * Math.sin(newTheta2);
//        double newTheta1 = Math.atan2(y, x) - Math.atan2(k2, k1);
//
//        if (Double.isNaN(newTheta1) || Double.isNaN(newTheta2) || Double.isInfinite(newTheta1) || Double.isInfinite(newTheta2)) {
//            return false;
//        }
//
//        armAngles.angle1 = newTheta1;
//        armAngles.angle2 = newTheta2;
//        return true;
//    }
//
//    public ArmAngles getClawPositionAngles(ClawPosition clawPosition) {
//        ArmAngles armAngles = new ArmAngles(clawPosition.coordinates, clawPosition.inverted);
//        return armAngles;
//    }

    /*
     * Return the angle difference travelling clockwise from startAngle to endAngle
     */
    public double getAngleDifferenceCounterClockwise(double startAngle, double endAngle) {
        if (startAngle <= endAngle) {
            return endAngle - startAngle;
        } else {
            return endAngle - startAngle + 2 * Math.PI;
        }
    }

    /*
     * Return the angle difference travelling clockwise from startAngle to endAngle
     */
    public double getSignedAngleDifferenceCounterClockwise(double startAngle, double endAngle, double threshold) {
        double difference = endAngle - startAngle;
        if (startAngle > endAngle) {
            difference += 2 * Math.PI;
        }
        if (difference > threshold) {
            difference -= 2 * Math.PI;
        }
        return difference;
    }

    /*
     * Store the parameters for a claw location
     * ClawX and ClawY are the coordinates of the claw.  The origin of the coordinate system is the pivot of the SCARA arm.
     * Positive X extends to the left side of the robot, parallel to the back plane of the robot.  Positive Y extends out
     * of the back of the robot
     * arm1Servo and arm2Servo are the observed servo values when the claw is at the given position.  To calibrate these
     * values, the claw should be moved to those coordinates using the test app.  The values displayed should be used here.
     * theta1 and theta2 are the calculated angles of the arms using the inverse kinematics equations.  theta1 is measured
     * relative to the claw's coordinate system above, with increasing angles moving counter-clockwise.  theta2 is measured
     * relative to the first arm.
     *
     * To calculate the servo position for the first arm, scale the servo value by the change in theta1, using two of these
     * control points to define the line.  To calculate the servo position for the second arm, scale the servo value by the change in
     * (theta2 - theta1), using two (possibly different) control points to define the line.
     */
    public class ClawPosition {
        Coordinates coordinates;
        ArmAngles armAngles;
        ServoPositions servoPositions;
        boolean inverted;

        public ClawPosition(double x, double y, double arm1Servo, double arm2Servo) {
            coordinates = new Coordinates(x, y);
            armAngles = new ArmAngles(coordinates, true);
            servoPositions = new ServoPositions(arm1Servo, arm2Servo);
            theta1 = theta2 = 0;
        }

        public ClawPosition(Coordinates coordinates, ServoPositions servoPositions) {
            this(coordinates.x, coordinates.y, servoPositions.servo1, servoPositions.servo2);
        }

        public ClawPosition(ClawPosition other) {
            this(other.coordinates, other.servoPositions);
            this.inverted = other.inverted;
        }

        // slow down the side to side motion to make it more useful
        public boolean moveBy(double deltaX, double deltaY, double deltaTime) {
            if (!coordinates.moveBy(deltaTime * ARM_SPEED * 0.5f * deltaX, deltaTime * ARM_SPEED  * 0.5f * deltaY)) return false;
            armAngles.setAngles(coordinates, true);
//            servoPositions.update(armAngles);
            servoPositions.updateFromControlAngles(armAngles);

            return true;
        }

        public boolean moveTo(double targetX, double targetY, double deltaTime) {
            double deltaX = targetX - coordinates.x;
            if (Math.abs(deltaX) > deltaTime * ARM_SPEED) deltaX = Math.signum(deltaX) * deltaTime * ARM_SPEED;

            double deltaY = targetY - coordinates.y;
            if (Math.abs(deltaY) > deltaTime * ARM_SPEED) deltaY = Math.signum(deltaY) * deltaTime * ARM_SPEED;

            if (!coordinates.moveBy(deltaX, deltaY)) return false;
            armAngles.setAngles(coordinates, true);
//            servoPositions.update(armAngles);
            servoPositions.updateFromControlAngles(armAngles);
            return true;
        }

        /*
         * Return whether destination has been reached
         */
        public boolean moveSequence(Sequence sequence, double deltaTime) {
            Coordinates start = sequence.coordinatesList.get(0);
            Coordinates end = sequence.coordinatesList.get(sequence.coordinatesList.size() -1);
            double distanceToStart = Math.hypot(start.x - coordinates.x, start.y - coordinates.y);
            double distanceToEnd = Math.hypot(end.x - coordinates.x, end.y - coordinates.y);
            double totalDistance = Math.hypot(end.x - start.x, end.y - start.y);
//telemetry.addData("distance to start", distanceToStart);
//telemetry.addData("distance to end", distanceToEnd);
//telemetry.addData("total distace", totalDistance);
            if (distanceToStart + distanceToEnd > 1.05 * totalDistance) { // too far off the line, so move to the start
                moveTo(start.x, start.y, deltaTime);
                return false;
            }

            double targetDistance = distanceToStart + deltaTime * ARM_SPEED;
//            telemetry.addData("target distace", targetDistance);
//            telemetry.addData("delta time", deltaTime);
//            telemetry.addData("delta distace", targetDistance - distanceToStart);
            if (targetDistance > totalDistance) { // move is done
                coordinates.copy(end);
                armAngles.copy(sequence.angleList.get(sequence.angleList.size() - 1));
                servoPositions.updateFromControlAngles(armAngles);
                return true;
            }
            double stepDistance = totalDistance / 10;
            int index = (int) (targetDistance / stepDistance);
            double scale = (targetDistance - index * stepDistance) / stepDistance;
//            telemetry.addData("stepDistance", stepDistance);
//            telemetry.addData("index", index);
//            telemetry.addData("scale", scale);

            // range check the index
//            if ((index < 0) || (index >= sequence.coordinatesList.size())) {
//                return true; // something's wrong so move on
//            }

            Coordinates p1 = sequence.coordinatesList.get(index);
            Coordinates p2 = sequence.coordinatesList.get(index + 1);
            coordinates.x = p1.x + (p2.x - p1.x) * scale;
            coordinates.y = p1.y + (p2.y - p1.y) * scale;

//            telemetry.addData("p1", p1.x+", "+p1.y);
//            telemetry.addData("p2", p2.x+", "+p2.y);
//            telemetry.addData("c", coordinates.x+", "+coordinates.y);

            ArmAngles a1 = sequence.angleList.get(index);
            ArmAngles a2 = sequence.angleList.get(index + 1);
            armAngles.angle1 = scaleAngleWithWraparound(a1.angle1, a2.angle1, scale);
            armAngles.angle2 = scaleAngleWithWraparound(a1.angle2, a2.angle2, scale);
//            armAngles.angle1 = a1.angle1 + (a2.angle1 - a1.angle1) * scale;
//            armAngles.angle2 = a1.angle2 + (a2.angle2 - a1.angle2) * scale;
            servoPositions.updateFromControlAngles(armAngles);
            return false;
        }

    }


    /*
     * Scale between the two angles accounting for wraparound
     */
    private double scaleAngleWithWraparound(double a1, double a2, double scale) {
        double diff = a2 - a1;
        if (Math.abs(diff) < Math.PI) return a1 + diff * scale; // no wraparound
        if (diff < 0) diff += 2 * Math.PI;
        else diff -= 2 * Math.PI;
        double result = a1 + diff * scale;
        if (result >= Math.PI) result -= 2 * Math.PI;
        if (result < -Math.PI) result += 2 * Math.PI;
        return result;
    }

    public class ArmAngles {
        double angle1;
        double angle2;

        public ArmAngles(double angle1, double angle2) {
            this.angle1 = angle1;
            this.angle2 = angle2;
        }

        public ArmAngles(ArmAngles other) {
            this(other.angle1, other.angle2);
        }

        public ArmAngles(Coordinates coordinates, boolean inverted) {
            setAngles(coordinates, inverted);
        }

        public boolean copy(ArmAngles other) {
            this.angle1 = other.angle1;
            this.angle2 = other.angle2;
            return true;
        }

        public boolean adjust(double deltaAngle1, double deltaAngle2) {
            angle1 += deltaAngle1;
            angle2 += deltaAngle2;
            return true;
        }

        public boolean setAngles(Coordinates coordinates, boolean inverted) {
            double x = coordinates.x;
            double y = coordinates.y;
            double cos_theta2 = (x*x + y*y - arm1Length * arm1Length - arm2Length * arm2Length) / (2 * arm1Length * arm2Length);

            // first term can be plus or minus resulting in two different solutions for the same position
            double newTheta2 = Math.atan2((inverted ? -1 : 1) * Math.sqrt(1 - cos_theta2 * cos_theta2), cos_theta2);
            double k1 = arm1Length + arm2Length * Math.cos(newTheta2);
            double k2 = arm2Length * Math.sin(newTheta2);
            double newTheta1 = Math.atan2(y, x) - Math.atan2(k2, k1);

            if (Double.isNaN(newTheta1) || Double.isNaN(newTheta2) || Double.isInfinite(newTheta1) || Double.isInfinite(newTheta2)) {
                return false;
            }

            angle1 = newTheta1;
            angle2 = newTheta2;
            return true;
        }

    }

    public class ServoPositions {
        double servo1;
        double servo2;

        public ServoPositions(double servo1, double servo2) {
            this.servo1 = servo1;
            this.servo2 = servo2;
        }

        public ServoPositions(ServoPositions other) {
            this(other.servo1, other.servo2);
        }

        public ServoPositions(ArmAngles armAngles) {
            update(armAngles);
        }

        /*
         * update the servo positions based on the given arm angles.  Return true if the arm angles are within the servos' range
         */
        public boolean update(ArmAngles armAngles) {
            boolean inRange = true;

            // use the position inside of the robot and under the bridge to scale the servo value;
            servo1 = clawInsideRobot.servoPositions.servo1 + getSignedAngleDifferenceCounterClockwise(clawInsideRobot.armAngles.angle1, armAngles.angle1, Math.PI) * servo1TicksPerRadian;
            if (servo1 > 1) {
                servo1 = 1;
                inRange = false;
            }
            if (servo1 < 0) {
                servo1 = 0;
                inRange = false;
            }

            // servo 2's position depends on servo 1's position
            // Theta2 is arm2's angle relative to arm1.  When arm 1's angle changes, servo 2 must rotate to keep the same angle between arm1 and arm2.  This is
            // because arm2's pivot is connected to the servo through a belt which rotates with arm 1.
            // Alternatively define theta2 relative to the frame.
            servo2 = clawInsideRobot.servoPositions.servo2 + getSignedAngleDifferenceCounterClockwise(clawInsideRobot.armAngles.angle1 + clawInsideRobot.armAngles.angle2, armAngles.angle1 + armAngles.angle2, Math.PI * .75)* servo2TicksPerRadian;
            if (servo2 > 1) {
                servo2 = 1;
                inRange = false;
            }
            if (servo2 < 0) {
                servo2 = 0;
                inRange = false;
            }

            return inRange;
        }
        public boolean updateFromReference(ArmAngles armAngles, ClawPosition p1, ClawPosition p2) {
            boolean inRange = true;

            // use the position inside of the robot and under the bridge to scale the servo value;
            servo1 = p1.servoPositions.servo1 + getSignedAngleDifferenceCounterClockwise(p1.armAngles.angle1, armAngles.angle1, Math.PI) * getServo1TicksPerRadian(p1, p2);
            if (servo1 > 1) {
                servo1 = 1;
                inRange = false;
            }
            if (servo1 < 0) {
                servo1 = 0;
                inRange = false;
            }

            // servo 2's position depends on servo 1's position
            // Theta2 is arm2's angle relative to arm1.  When arm 1's angle changes, servo 2 must rotate to keep the same angle between arm1 and arm2.  This is
            // because arm2's pivot is connected to the servo through a belt which rotates with arm 1.
            // Alternatively define theta2 relative to the frame.
            servo2 = p1.servoPositions.servo2 + getSignedAngleDifferenceCounterClockwise(p1.armAngles.angle1 + p1.armAngles.angle2, armAngles.angle1 + armAngles.angle2, Math.PI * .75)* getServo2TicksPerRadian(p1, p2);
            if (servo2 > 1) {
                servo2 = 1;
                inRange = false;
            }
            if (servo2 < 0) {
                servo2 = 0;
                inRange = false;
            }

            return inRange;
        }

        public boolean updateFromControlAngles(ArmAngles armAngles) {
            boolean inRange = true;

            // use the position inside of the robot and under the bridge to scale the servo value;
            servo1 = ARM1_SERVO_POSITION_270_DEGREES + getSignedAngleDifferenceCounterClockwise(Math.PI * 1.5, armAngles.angle1, Math.PI) *
                    (ARM1_SERVO_POSITION_270_DEGREES - ARM1_SERVO_POSITION_90_DEGREES) / Math.PI;
            if (servo1 > 1) {
                servo1 = 1;
                inRange = false;
            }
            if (servo1 < 0) {
                servo1 = 0;
                inRange = false;
            }

            // servo 2's position depends on servo 1's position
            // Theta2 is arm2's angle relative to arm1.  When arm 1's angle changes, servo 2 must rotate to keep the same angle between arm1 and arm2.  This is
            // because arm2's pivot is connected to the servo through a belt which rotates with arm 1.
            // Alternatively define theta2 relative to the frame.
            double angleDiff = getSignedAngleDifferenceCounterClockwise(Math.PI * 0.5, armAngles.angle1 + armAngles.angle2, Math.PI);
            servo2 = ARM2_SERVO_POSITION_90_DEGRESS - angleDiff * (ARM2_SERVO_POSITION_90_DEGRESS - ARM2_SERVO_POSITION_180_DEGREES) / (Math.PI * 0.5);
            telemetry.addData("angle diff", angleDiff);
            telemetry.addData("a1+a2", armAngles.angle1 + armAngles.angle2);
telemetry.addData("s2 tivks per Rad", (ARM2_SERVO_POSITION_90_DEGRESS - ARM2_SERVO_POSITION_180_DEGREES) / (Math.PI * 0.5));
            if (servo2 > 1) {
                servo2 = 1;
                inRange = false;
            }
            if (servo2 < 0) {
                servo2 = 0;
                inRange = false;
            }

            return inRange;
        }

    }

    public class Coordinates {
        double x, y;

        public Coordinates(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Coordinates(Coordinates other) {
            this(other.x, other.y);
        }

        public boolean copy(Coordinates other) {
            this.x = other.x;
            this.y = other.y;
            return true;
        }

        /*
         * Allowable poositions are within 10mm of the line from clawInsideRobot to clawOutsideRobot or
         * within 10mm of the line from clawOutsideRobot - SIDE_TO_SIDE_RANGE/2 to clawOutsideRobot + SIDE_TO_SIDE_RANGE/2
         */
        public boolean isValid(double newX, double newY) {
            if ((newY >= clawInsideRobot.coordinates.y) && (newY <= DELIVER_Y_DISTANCE) &&
                    (newX >= clawInsideRobot.coordinates.x - 30) && (newX <= clawOutsideRobot.coordinates.x + 30)) return true;
            if ((newY <= DELIVER_Y_DISTANCE + 70) && (newY >= DELIVER_Y_DISTANCE - 70) &&
                    (newX >= MIDLINE - SIDE_TO_SIDE_RANGE * 0.5) && (newX <= MIDLINE + SIDE_TO_SIDE_RANGE * 0.5)) return true;
            return false;
        }

        public boolean moveBy(double deltaX, double deltaY) {
            double newX = x + deltaX;
            double newY = y + deltaY;
            if (!isValid(newX, newY)) return false;
            x = newX;
            y = newY;
            return true;
        }
    }


    ArrayList<ArmAngles> angleList;
    double sequenceStartTime;

    public boolean setSequence(ArrayList<ArmAngles> angleList, double currentTime) {
        this.angleList = angleList;
        sequenceStartTime = currentTime;
        return true;
    }

    public class Sequence {
        ArrayList<Coordinates> coordinatesList;
        ArrayList<ArmAngles> angleList;
        public Sequence(Coordinates start, Coordinates end) {
            coordinatesList = new ArrayList<Coordinates>();
            angleList = new ArrayList<ArmAngles>();

            for (int i = 0; i <= 10; i++) {
                Coordinates coord = new Coordinates(start.x + (end.x - start.x) * 1.0 * i / 10, start.y + (end.y - start.y) * 1.0 * i /10);
                coordinatesList.add(coord);
                angleList.add(new ArmAngles(coord, true));
            }

        }
    }
}