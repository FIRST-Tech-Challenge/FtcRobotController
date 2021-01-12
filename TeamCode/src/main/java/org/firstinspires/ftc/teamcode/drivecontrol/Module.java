package org.firstinspires.ftc.teamcode.drivecontrol;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Module {
    private boolean debug;
    private Log log;

    private Drive.ModuleSide side;

    enum ModuleRotationMode {
        ROTATION_ENABLED, ROTATION_DISABLED
    }

    private ModuleRotationMode rotationMode;

    private DcMotor topMotor;
    private DcMotor bottomMotor;

    private Telemetry telemetry;
    private Vector2D positionVector;
    public double positionChange;

    private final Vector2D TOP_MOTOR_VECTOR = new Vector2D(1/Math.sqrt(2), 1/Math.sqrt(2));
    private final Vector2D BOTTOM_MOTOR_VECTOR = new Vector2D(-1/Math.sqrt(2), 1/Math.sqrt(2));
    public static final double MAX_POWER = 1.0;
    public static final double MAX_ANGLE = 60;
    public static final double ORIENTATION_ERROR_MARGIN = 5;
    public static final double ROT_ADVANTAGE = 1.7; //TODO: Tune
    public static final double TICKS_PER_MODULE_REV = 1014;
    public static final double DEGREES_PER_TICK = 360/TICKS_PER_MODULE_REV;
    public static final double TICKS_PER_WHEEL_REV = 1 * TICKS_PER_MODULE_REV * 18/60; //ticks per WHEEL revolution
    public static final double CM_WHEEL_DIAMETER = 2.5 * 2.54;
    public static final double CM_PER_WHEEL_REV = CM_WHEEL_DIAMETER * Math.PI;
    public static final double CM_PER_TICK = CM_PER_WHEEL_REV/TICKS_PER_WHEEL_REV;

    private boolean reversed = false;
    private boolean takingShortestPath = false;

    private double distanceTraveled = 0;
    private double lastTopEncoder = 0;
    private double lastBottomEncoder = 0;

    public Module(boolean debug, Drive.ModuleSide side, DcMotor topMotor, DcMotor bottomMotor, Telemetry telemetry) {
        this.debug = debug;

        this.side = side;

        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        if (side == Drive.ModuleSide.RIGHT) {
            positionVector = new Vector2D((double)18/2, 0); //points from robot center to right module
        } else {
            positionVector = new Vector2D((double)-18/2, 0); //points from robot center to left module
        }

        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (debug) {
            log = new Log(side + "ModuleLog");
            log.addField("Trans Vector FC X");
            log.addField("Trans Vector FC Y");
            log.addField("Rot Vector X");
            log.addField("Rot Vector Y");
            log.addField("Target Vector X");
            log.addField("Target Vector Y");
            log.addField("Module Orientation");
            log.addField("Reversed");
            log.addField("Power Vector X (TRANS)");
            log.addField("Power Vector Y (ROT)");
            log.addField("Motor 1 Power");
            log.addField("Motor 2 Power");
            log.addField("Motor 1 Encoder");
            log.addField("Motor 2 Encoder");
            log.newLine();
        }
    }

    public Module(Drive.ModuleSide side, DcMotor topMotor, DcMotor bottomMotor, Telemetry telemetry) {
        this(false, side, topMotor, bottomMotor, telemetry);
    }

    public void setRotationMode(ModuleRotationMode mode) {
        rotationMode = mode;
    }

    public void updateTarget(Vector2D translationVector, double rotationMagnitude, double heading) {
        Vector2D rotatedTranslationVector = translationVector.rotateBy(heading, Vector2D.CCW);
        Vector2D rotationVector = positionVector.normalize(rotationMagnitude).rotateBy(90, Vector2D.CCW);

        Vector2D targetVector = rotatedTranslationVector.add(rotationVector);

        int directionMultiplier = -1;
        if (reversed) {
            targetVector = targetVector.reflect();
            directionMultiplier = 1;
        }

        goToTarget(targetVector, directionMultiplier, heading);

        if (debug) {
            log.addField(rotatedTranslationVector.getX());
            log.addField(rotatedTranslationVector.getY());
            log.addField(rotationVector.getX());
            log.addField(rotationVector.getY());
            log.addField(targetVector.getX());
            log.addField(targetVector.getY());
            log.addField(heading);
            log.addField(reversed);

            telemetry.addData(side + " REVERSED: ", reversed);
            telemetry.addData(side + " Trans Vec FC: ", rotatedTranslationVector);
            telemetry.addData(side + " Rot Vec: ", rotationVector);
        }
    }

    private void goToTarget(Vector2D target, int directionMultiplier, double heading) {
        double moveComponent = target.getMagnitude() * directionMultiplier;

        double pivotComponent;
        if (target.getMagnitude() != 0) {
            pivotComponent = getPivotComponent(target, heading);
        } else {
            pivotComponent = 0;
        }

        Vector2D powerVector = new Vector2D(moveComponent, pivotComponent);

        if (debug) {
            log.addField(powerVector.getX());
            log.addField(powerVector.getY());
        }

        setMotorPowers(powerVector);

        if (debug) {
            telemetry.addData(side + " Target Vector Angle: ", target.getAngle());
            telemetry.addData(side + " Power Vector: ", powerVector);
            telemetry.addData(side + " Current orientation: ", heading);
            log.newLine();
        }
    }

    private double getPivotComponent(Vector2D target, double currentAngle) {
        double targetAngle = target.getAngle();
        double angleDifference;
        double rawDiff = Math.abs(currentAngle - targetAngle);
        if (rawDiff > 180) {
            angleDifference = 360 - rawDiff;
        } else {
            angleDifference = rawDiff;
        }

        if (angleDifference > 110) {
            if (!takingShortestPath) {
                reversed = !reversed;
            }
            takingShortestPath = true;
        } else {
            takingShortestPath = false;
        }

        if (rotationMode == ModuleRotationMode.ROTATION_DISABLED) {
            return 0;
        }

        int direction = directionTo(currentAngle, targetAngle);

        if (angleDifference < ORIENTATION_ERROR_MARGIN) {
            return 0;
        } else if (angleDifference > MAX_ANGLE) {
            if (direction == Vector2D.CW) {
                return ROT_ADVANTAGE;
            } else {
                return -1 * ROT_ADVANTAGE;
            }
        } else {
            if (direction == Vector2D.CW) {
                return angleDifference / MAX_ANGLE * ROT_ADVANTAGE;
            } else {
                return -1 * angleDifference / MAX_ANGLE * ROT_ADVANTAGE;
            }
        }
    }

    private void setMotorPowers(Vector2D powerVector) {
        Vector2D topMotorUnscaled = powerVector.projection(TOP_MOTOR_VECTOR);
        Vector2D bottomMotorUnscaled = powerVector.projection(BOTTOM_MOTOR_VECTOR);

        Vector2D[] motorPowersScaled = Vector2D.batchNormalize(MAX_POWER, topMotorUnscaled, bottomMotorUnscaled);
        double topMotorPower = motorPowersScaled[0].getMagnitude();
        double bottomMotorPower = motorPowersScaled[1].getMagnitude();

        if (motorPowersScaled[0].getAngle() != TOP_MOTOR_VECTOR.getAngle()) {
            topMotorPower *= -1;
        }
        if (motorPowersScaled[1].getAngle() != BOTTOM_MOTOR_VECTOR.getAngle()) {
            bottomMotorPower *= -1;
        }

        topMotor.setPower(topMotorPower);
        bottomMotor.setPower(bottomMotorPower);
    }

    private int directionTo(double angleFrom, double angleTo) {
        double rawDiff = Math.abs(angleTo - angleFrom);
        if (rawDiff > 180) {
            if (angleTo > angleFrom) {
                return Vector2D.CW;
            } else {
                return Vector2D.CCW;
            }
        } else {
            if (angleTo > angleFrom) {
                return Vector2D.CCW;
            } else {
                return Vector2D.CW;
            }
        }
    }

    public void updateTracking () {
        double currentTopEncoder = topMotor.getCurrentPosition();
        double currentBottomEncoder = bottomMotor.getCurrentPosition();

        double topMotorChange = currentTopEncoder - lastTopEncoder;
        double bottomMotorChange = currentBottomEncoder - lastBottomEncoder;

        //if module is reversed, subtract distance traveled instead of adding
        //module is driving in the opposite direction that the encoders "think" it is
        if (reversed) {
            distanceTraveled -= (topMotorChange - bottomMotorChange)/2.0 * CM_PER_TICK;
        } else {
            distanceTraveled += (topMotorChange - bottomMotorChange)/2.0 * CM_PER_TICK;
        }

        lastTopEncoder = currentTopEncoder;
        lastBottomEncoder = currentBottomEncoder;

        telemetry.addData(side + "Motor 1 Encoder", currentTopEncoder);
        telemetry.addData(side + "Motor 2 Encoder", currentBottomEncoder);
        telemetry.update();
    }

    public Vector2D updatePositionTracking() {
        double topEncoder = topMotor.getCurrentPosition();
        double bottomEncoder = bottomMotor.getCurrentPosition();

        double startAngle = ((lastTopEncoder + lastBottomEncoder)/2.0 * DEGREES_PER_TICK) % 360;
        double finalAngle = ((topEncoder + bottomEncoder)/2.0 * DEGREES_PER_TICK) % 360;
        double averageAngle = Math.toRadians(getAverageAngle(startAngle, finalAngle)); //was 180 heading TODO check

        double startingPosition = (lastTopEncoder - lastBottomEncoder)/2.0  * CM_PER_TICK;
        double finalPosition = (topEncoder - bottomEncoder)/2.0 * CM_PER_TICK;
        positionChange = finalPosition - startingPosition;

        Vector2D displacementVec;
        double deltaYPos = Math.sin(averageAngle) * positionChange;
        double deltaXPos = Math.cos(averageAngle) * positionChange;
        displacementVec = new Vector2D(-deltaXPos, -deltaYPos);

        if (debug) {
            telemetry.addData("Position change: ", positionChange);
            telemetry.addData("Average angle: ", averageAngle);
            telemetry.addData(side + " Displacement vector: ", displacementVec);
            telemetry.addData(side + " Delta X Pos: ", displacementVec.getX());
            telemetry.addData(side + " Delta Y Pos: ", displacementVec.getY());
        }

        lastTopEncoder = topEncoder;
        lastBottomEncoder = bottomEncoder;

        return displacementVec;
    }

    public double getAverageAngle(double startAngle, double finalAngle) {
        double raw = startAngle - finalAngle;
        double diff = 0;
        if (raw > 180) {
            diff = 360 - raw;
            if (finalAngle > startAngle) {
                return startAngle + (diff / 2.0);
            } else {
                return startAngle - (diff / 2.0);
            }
        } else {
            if (finalAngle > startAngle) {
                return startAngle - (raw / 2.0);
            } else {
                return startAngle + (raw / 2.0);
            }
        }


    }

    public double getCurrentOrientation() {
        double rawAngle = (double)(bottomMotor.getCurrentPosition() + topMotor.getCurrentPosition())/2.0 * DEGREES_PER_TICK;
        return rawAngle % 360;
    }

    public void rotateModule (Vector2D direction, boolean fieldCentric, double heading) {
        Vector2D directionVector = direction.rotateTo(heading); //was converted robot heading

        if (reversed) {
            directionVector = directionVector.reflect();
            direction = direction.reflect();
        }

        Vector2D powerVector;
        if (fieldCentric) {
            powerVector = new Vector2D(0, getPivotComponent(directionVector, getCurrentOrientation())); //order important here
        } else {
            powerVector = new Vector2D(0, getPivotComponent(direction, getCurrentOrientation())); //order important here
        }
        setMotorPowers(powerVector);

        if (debug) {
            telemetry.addData(side + " Power Vector: ", powerVector);
            telemetry.addData(side + " Current orientation: ", getCurrentOrientation());
        }
    }

    public double getDistanceTraveled() {
        return distanceTraveled;
    }

    public void resetDistanceTraveled() {
        distanceTraveled = 0;
        lastTopEncoder = topMotor.getCurrentPosition();
        lastBottomEncoder = bottomMotor.getCurrentPosition();
    }
}
