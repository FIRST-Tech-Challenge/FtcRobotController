package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Robot {
    //SERVO OBJECTS
    Servo latchServo1, latchServo2;
    Servo grabberServo;
    Servo hungryHippoServo;
    Servo tapeMeasureServo;

    Servo outtake1, outtake2, outtakeRot;
    Servo capstone;

    //MOTORS
    ExpansionHubMotor lift1, lift2;
    ExpansionHubMotor intake1, intake2;

    DriveController driveController;
    BNO055IMU imu;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    OpMode opMode;

    //SENSORS
//    DistanceSensor frontRangeSensor;
    DistanceSensor backRangeSensor;

    //BULK DATA (RevExtensions2)
    RevBulkData bulkData1;
    RevBulkData bulkData2;
    ExpansionHubEx expansionHub1;
    ExpansionHubEx expansionHub2;

    //data logger
    DataLogger dataLogger;

    SCARAController controller;
    SCARAController.ClawPosition currentClawPosition;

    final DcMotor.RunMode DEFAULT_RUN_MODE;
    final boolean IS_AUTO;
    int targetPosLift;
    double LIFT_TICKS_PER_MS = 50.0/30.0;

    boolean wasAtZero = true; //assume the lift always starts unpowered?
    double lastTimeLift;
    //boolean IMUReversed = false;

    // only 8 levels are valid
    int[] encoderTicksAtLiftPositions = /*new int[8]*/ {0, 155, 309, 461, 689, 883, 1223, 1603};
    int liftPosition = 0;
    boolean wasLastPositive = false;

    //lift constants
    //final double barLength = 76.8; //units: cm
    //final int ticksAtStraight = 0;
//    final double liftStartHeight = 4.60375, barLength = 38.4, blockHeight = 10.16;
    // adjusted the lowest height to 4.3cm (measured on the robot
    final double liftStartHeight = 4.3, barLength = 38.4, blockHeight = 10.16;
    final double startAngle = Math.asin(liftStartHeight / barLength);
    final double ticksPerEntireRotation = 28*13.7*302.0/14;
    final double twoPi = 2 * Math.PI;

    //keeps track of outtake location
    private double outtakeX;
    private double outtakeY;
    private double outtakeDelay;
    private boolean gripperVertical;
    public double outatkeRotatePosition;


    public Robot (OpMode opMode, Position startingPosition, boolean isAuto, boolean debuggingMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        driveController = new DriveController(this, startingPosition, debuggingMode);
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu 1");

        IS_AUTO = isAuto;
        DEFAULT_RUN_MODE = isAuto ? AUTO_RUN_MODE : TELEOP_RUN_MODE;

        //bulk data
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        bulkData1 = expansionHub1.getBulkInputData();
        bulkData2 = expansionHub2.getBulkInputData();

        //the cooper thing
        latchServo1 = hardwareMap.servo.get("latchServo1");
        setupServo(latchServo1);
        latchServo2 = hardwareMap.servo.get("latchServo2");

        setupServo(latchServo2);

        //the cyrus thing
        hungryHippoServo = hardwareMap.servo.get("hungryHippoServo");
        setupServo(hungryHippoServo);

        tapeMeasureServo = hardwareMap.servo.get("tapeMeasureServo"); //change name
        setupServo(tapeMeasureServo);

        //the cooper things
        capstone = hardwareMap.servo.get("capstoneServo");
        setupServo(capstone);


        outtake1 = hardwareMap.servo.get("servoLink1");
        outtake2 = hardwareMap.servo.get("servoLink2");
        outtakeRot = hardwareMap.servo.get("gripperRotation");


        outatkeRotatePosition = 0.397;
        outtakeRot.setPosition(outatkeRotatePosition);
        gripperVertical = true;

        controller = new SCARAController(120, 120, telemetry);
        currentClawPosition = controller.new ClawPosition(controller.clawInsideRobot);

        grabberServo = hardwareMap.servo.get("grabberServo");
        setupServo(grabberServo);

        setupLift();

        intake1 = (ExpansionHubMotor) hardwareMap.dcMotor.get("intakeMotor1");
        setupMotor(intake1, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        intake2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("intakeMotor2");
        setupMotor(intake2, DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);

//        frontRangeSensor = hardwareMap.get(DistanceSensor.class, "frontRangeSensor");
        backRangeSensor = hardwareMap.get(DistanceSensor.class, "backRangeSensor");

        dataLogger = new DataLogger("SkystoneRobot");
    }

    //defaults to debugging mode off, starting position of 0, 0
    public Robot (OpMode opMode, boolean isAuto) {
        this(opMode, new Position(0, 0, new Angle(0, Angle.AngleType.ZERO_TO_360_HEADING)), isAuto,false);
    }

    //defaults to starting position of 0, 0
    public Robot (OpMode opMode, boolean isAuto, boolean debuggingMode) {
        this(opMode, new Position(0, 0, new Angle(0, Angle.AngleType.ZERO_TO_360_HEADING)), isAuto, debuggingMode);
    }

    public void updateBulkData () {
        bulkData1 = expansionHub1.getBulkInputData();
        bulkData2 = expansionHub2.getBulkInputData();
    }

    public void initIMU () {
        //this.IMUReversed = reversed;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    //public void initIMU () { initIMU(IMUReversed); }

    public Angle getRobotHeading () {
        //heading is of NEG_180_TO_180_HEADING type by default (no need for conversion)
        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

//        if (IMUReversed) {
//            return new Angle(heading-180, Angle.AngleType.NEG_180_TO_180_HEADING);
//        }
        //todo: check if heading should be negative or not
        return new Angle(-heading, Angle.AngleType.NEG_180_TO_180_HEADING);
    }

    public double getRobotHeadingDouble (Angle.AngleType type){
        return getRobotHeading().convertAngle(type).getAngle();
    }

    //SETUP METHODS
    public void setupMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setMode(DEFAULT_RUN_MODE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //takes RunMode parameter
    public void setupMotor(DcMotor motor, DcMotor.Direction direction, DcMotor.RunMode RUN_MODE) {
        motor.setDirection(direction);
        motor.setMode(RUN_MODE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setupServo(Servo servo) {
        servo.setDirection(DEFAULT_SERVO_DIRECTION);
    }

    //ANGLE METHODS
//    public Orientation getAngleOrientation() { return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); }
//    public double getCurrentAngleZ() { return getAngleOrientation().firstAngle; }
//    public double getCurrentAngleY() { return getAngleOrientation().secondAngle; }
//    public double getCurrentAngleX() { return getAngleOrientation().thirdAngle; }
//    public static double getZAngle(Orientation o) { return o.firstAngle; }
//    public static double getYAngle(Orientation o) { return o.secondAngle; }
//    public static double getXAngle(Orientation o) { return o.thirdAngle; }
//    public static double wrapAngle(double angle) { return angle < 0 ? angle % (2 * Math.PI) + 2 * Math.PI : angle % (2 * Math.PI); }

    //SERVOS
    public void moveServo(Servo servo, double pos) { servo.setPosition(pos); }

    public void unlatch() {
        moveServo(latchServo1, UNLATCHED_POSITION_1);
        moveServo(latchServo2, UNLATCHED_POSITION_2);
    }

    public void latch() {
        moveServo(latchServo2, LATCHED_POSITION_2);
        moveServo(latchServo1, LATCHED_POSITION_1);
    }

    public void closeGrabber() {
        moveServo(grabberServo, GRABBER_CLOSE_POSITION);
    }
    public void openGrabber() {
        moveServo(grabberServo, GRABBER_OPEN_POSITION);
    }

    public void moveGrabberToMid() {
        grabberServo.setPosition(0.6);
    }

    public void hungryHippoExtend(){
        moveServo(hungryHippoServo, HUNGRY_HIPPO_EXTEND_POSITION);
    }
    public void hungryHippoRetract(){
        moveServo(hungryHippoServo, HUNGRY_HIPPO_RETRACT_POSITION);
    }

    /*
    public void intakeServoOpen(){
        moveServo(intakeServo1, 0);
        moveServo(intakeServo2, 1);
    }
    public void intakeServoClose(){
        moveServo(intakeServo1, 1);
        moveServo(intakeServo2, 0);
    }
    */

    //MOTORS
    public void moveMotor(DcMotor motor, double power) { motor.setPower(power); }
    /*
        public void setArmPower(double power){
            if (power > 0.1) {
                armServo1.setPosition(1);
                armServo2.setPosition(0);
            } else if (power < -0.1) {
                armServo1.setPosition(0);
                armServo2.setPosition(1);
            } else {
                armServo1.setPosition(0.5);
                armServo2.setPosition(0.5);
            }
    //        moveServo(armServo1, servoPowerCalc(power));
    //        moveServo(armServo2, servoPowerCalc(-power));
    //        telemetry.addData( "outtake 1: ", servoPowerCalc(power));
    //        telemetry.addData("outtake 2: ", servoPowerCalc(-power));
    //        telemetry.update();
        }
    */
    public double determineBlockPlacementTicks(int block) {
        return ((Math.asin((liftStartHeight + (blockHeight / 2) * block) / barLength) - startAngle) / (twoPi)) * ticksPerEntireRotation;
    }

    public double scaleLiftRate(double initalRate, double deltaTime) {
        //scaledRate = RobotUtil.scaleVal(initalRate, 0, 1, )

        if (initalRate < 0) {
            // go slower on the way down (or not)
            return initalRate * 75;
        } else {
            return initalRate * 50;
        }
    }

    public void setupLift() {
        //the obvious things
        lift1 = (ExpansionHubMotor) hardwareMap.dcMotor.get("liftMotorLeft");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setTargetPosition(bulkData1.getMotorCurrentPosition(lift1));
        setupMotor(lift1, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_TO_POSITION);

        lift2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("liftMotorRight");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setTargetPosition(bulkData1.getMotorCurrentPosition(lift2));
        setupMotor(lift2, DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_TO_POSITION);

//        for (int i = 0; i < encoderTicksAtLiftPositions.length; i++) {
//            encoderTicksAtLiftPositions[i] = (int)determineBlockPlacementTicks(i);
//        }
//        // limit the top position to the physical constraints of the lift
//        encoderTicksAtLiftPositions[encoderTicksAtLiftPositions.length - 1] = (int)(((Math.asin(73.10 /*max height*// 2 / barLength) - startAngle) / (twoPi)) * ticksPerEntireRotation);
    }

    public int getLiftBlockPositionToGo(int position, boolean goingUp) {
        if (goingUp) {
            for (int i = 0; i < encoderTicksAtLiftPositions.length; i++) {
                if (position < encoderTicksAtLiftPositions[i]) {
                    return encoderTicksAtLiftPositions[i];
                }
            }
            return encoderTicksAtLiftPositions[encoderTicksAtLiftPositions.length - 1];
        } else {
            for (int i = encoderTicksAtLiftPositions.length - 1; i >= 0; i--) {
                if (position > encoderTicksAtLiftPositions[i]) {
                    return encoderTicksAtLiftPositions[i];
                }
            }
            return encoderTicksAtLiftPositions[0];
        }
    }

    public void moveLift(double moveRate) {

        double deltaTime = System.currentTimeMillis() - lastTimeLift;
        // don't reset static variable wasAtZero every loop
        //wasAtZero = true;

        int lift1CurrentPos = bulkData1.getMotorCurrentPosition(lift1);
        int lift2CurrentPos = bulkData1.getMotorCurrentPosition(lift2);
        //int position = (lift1CurrentPos + lift2CurrentPos)/2; //integer division, i know (i think it's fine)
        int position = lift1CurrentPos;

        if (Math.abs(moveRate) < 0.1) {
            moveRate = 0;
        } else {
            wasAtZero = false;
        }

        //double scaledMoveRate = scaleLiftRate(moveRate, deltaTime);
        double scaledMoveRate = moveRate * LIFT_TICKS_PER_MS * deltaTime;
        // if (scaledMoveRate < 0) scaledMoveRate *=2; // lift will go down faster than up
        //double scaledMoveRate = moveRate * 60;
        if (moveRate == 0) {
            if (!wasAtZero) {
                wasAtZero = true;
                //position = (lift1CurrentPos + lift2CurrentPos)/2;
                position = getLiftBlockPositionToGo(lift1CurrentPos, wasLastPositive);

                // hold on to the last integral block level as the target position
                targetPosLift = position;

            }
        } else {
            // only update the target position if the move rate is non-zero
            targetPosLift += scaledMoveRate;
            // limit check
            if (targetPosLift < 0) targetPosLift = 0;
            // what is the maximum limit
        }

        if ((lift1CurrentPos < encoderTicksAtLiftPositions[1] * 0.25) &&
                (targetPosLift < encoderTicksAtLiftPositions[1] * 0.25)){ // lift is settling to the ground state so turn off power
            // this height should be ~1 inch above the lowest height so the lift should still be below the 14" bars
            lift1.setPower(0);
            lift2.setPower(0);
        } else { // otherwise use full power to move to and hold the elevated position
            if (lift1.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (lift2.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setTargetPosition(targetPosLift);
            lift2.setTargetPosition(targetPosLift);
            if (targetPosLift ==0) { // use less power when going down
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            } else {
                lift1.setPower(1);
                lift2.setPower(1);
            }
        }

        lastTimeLift = System.currentTimeMillis();

        wasLastPositive = moveRate > 0;

        telemetry.addData("Target Position for Lift", targetPosLift);
        telemetry.addData("Current Lift Power", scaledMoveRate);
        telemetry.addData("Loop time", deltaTime);
        telemetry.addData("Lift ticks/ms", LIFT_TICKS_PER_MS);
    }

    public void moveLiftToPosition (int position) {
        lift1.setPower(1);
        lift2.setPower(1);
        lift1.setTargetPosition(position);
        lift2.setTargetPosition(position);
    }

    public void moveIntake(IntakeState state) {
        //Defaults to being fast
        moveIntake(state, IntakeSpeed.FAST);
    }

    public Double getDistance(double wantedAngle) {

        return  (Math.abs(getRobotHeading().getAngle() - wantedAngle) < 5) ? //If within
                backRangeSensor.getDistance(DistanceUnit.CM) :
                null;

    }

    public void moveIntakeUntilStalled(IntakeState state, IntakeSpeed speed, long timeout, LinearOpMode linearOpMode) {
        moveIntake(state, speed);
        long startTime = System.currentTimeMillis();
        double previousEncoder = intakeEncoderValue();
        double currentEncoder = previousEncoder + Constants.MIN_ENCODER_DIFFERENCE_INTAKE + 1;
        while (currentEncoder - previousEncoder > Constants.MIN_ENCODER_DIFFERENCE_INTAKE && System.currentTimeMillis() - startTime < timeout && linearOpMode.opModeIsActive()) {
            telemetry.addData("Encoder change", currentEncoder - previousEncoder);
            telemetry.update();
            wait(200, linearOpMode);
            previousEncoder = currentEncoder;
            currentEncoder = intakeEncoderValue();
        }
    }

    public double intakeEncoderValue() {
        return (bulkData1.getMotorCurrentPosition(intake1) + bulkData1.getMotorCurrentPosition(intake2)) / 2.0;
    }

    public void moveIntake(IntakeState state, IntakeSpeed speed) {
        if (state == IntakeState.INTAKE) {
            if (speed == IntakeSpeed.FAST) {
                intake1.setPower(INTAKE_POWER_FAST);
                intake2.setPower(INTAKE_POWER_FAST);
            } else {
                intake1.setPower(INTAKE_POWER_SLOW);
                intake2.setPower(INTAKE_POWER_SLOW);
            }
        } else if (state == IntakeState.OUTTAKE) {
            if (speed == IntakeSpeed.FAST) {
                intake1.setPower(OUTTAKE_POWER_FAST);
                intake2.setPower(OUTTAKE_POWER_FAST);
            } else {
                intake1.setPower(OUTTAKE_POWER_SLOW);
                intake2.setPower(OUTTAKE_POWER_SLOW);
            }
        } else {
            intake1.setPower(STOP_POWER);
            intake2.setPower(STOP_POWER);
        }
    }

    //manage all outtake movements in a single method, simplifying commands and not requiring a separate thread
    public void moveOuttake(double x_input, double y_input, boolean toDeploy, boolean toRetract){
        Outtake outtake = new Outtake();

        double[] servoPos;


        //manual command
        if(x_input != 0 || y_input != 0) {
            servoPos = outtake.processMovement(outtakeX, outtakeY, x_input, y_input);
            outtake1.setPosition(servoPos[0]);
            outtake2.setPosition(servoPos[1]);
        }
        else if(toDeploy) {
            if (System.currentTimeMillis() > outtakeDelay) {
                //check if gripper is oriented the wrong way
                if (!gripperVertical) {
                    orientGripper();
                    outtakeDelay = System.currentTimeMillis() + 500;
                }
                //if deploy sequence has not finished
                else if (outtakeY < outtake.getDEFAULT_EXTENSION()) {
                    servoPos = outtake.getDeployServoPos(outtakeY);
                    outtake1.setPosition(servoPos[0]);
                    outtake2.setPosition(servoPos[1]);
                    outtakeDelay = System.currentTimeMillis() +
                            (outtakeY - outtake.getPICKUP_OFFSET()) % outtake.getSTEP_INTERVAL();
                }
                //get in place for flip
                else if (outtakeX != -20 || outtakeY != 215) {
                    servoPos = outtake.getReverseKinematics(-20, 215, true);
                    outtake1.setPosition(servoPos[0]);
                    outtake2.setPosition(servoPos[1]);
                    outtakeDelay = System.currentTimeMillis() + Math.hypot(outtakeX - 20, outtakeY - 215) * 10;
                }
                //flipping states
                else if (outtake1.getPosition() > outtake2.getPosition()) {
                    double temp = outtake1.getPosition();
                    outtake1.setPosition(outtake2.getPosition());
                    outtake2.setPosition(temp);
                }
            }
        }
        else if(toRetract) {
            if (System.currentTimeMillis() > outtakeDelay) {
                //check if gripper is oriented the wrong way
                if (!gripperVertical) {
                    orientGripper();
                    outtakeDelay = System.currentTimeMillis() + 500;
                }
                //flip states if required
                if (outtake1.getPosition() < outtake2.getPosition()) {
                    //makes sure the outtake is in flippable location
                    if (outtakeX != -20 || outtakeY != 215) {
                        servoPos = outtake.getReverseKinematics(-20, 215, true);
                        outtake1.setPosition(servoPos[0]);
                        outtake2.setPosition(servoPos[1]);
                        outtakeDelay = System.currentTimeMillis() + Math.hypot(outtakeX - 20, outtakeY - 215) * 10;
                    }
                    double temp = outtake1.getPosition();
                    outtake1.setPosition(outtake2.getPosition());
                    outtake2.setPosition(temp);
                }
                //finish up retracting sequence
                else if (outtakeY > outtake.getPICKUP_OFFSET()) {
                    servoPos = outtake.getRetractServoPos(outtakeY);
                    outtake1.setPosition(servoPos[0]);
                    outtake2.setPosition(servoPos[1]);
                    outtakeDelay = System.currentTimeMillis() +
                            (outtakeY - outtake.getPICKUP_OFFSET()) % outtake.getSTEP_INTERVAL();
                }
            }
        }
    }

    public void orientGripper(){
        if(gripperVertical)
            outtakeRot.setPosition(1);
        else outtakeRot.setPosition(0);
    }

    public void initializeCapstone () {
        capstone.setPosition(0); //was 1
    }

    public void dropCapstone () {
        capstone.setPosition(1); //was 0
    }

    public void moveSingleIntakeRoller(boolean roller1) {
        (roller1 ? intake1 : intake2).setPower(INTAKE_POWER_SLOW);
    }

    public double servoPowerCalc(double p){
        return (p / 2.0) + .5;
    }

    public void wait (int millis, LinearOpMode linearOpMode) {
        long startTime = System.currentTimeMillis();
        while (millis > System.currentTimeMillis() - startTime && linearOpMode.opModeIsActive()) {}
    }

    public double getRange (boolean frontSensor) {
        //if (frontSensor) return frontRangeSensor.getDistance(DistanceUnit.CM);
        //else
        return backRangeSensor.getDistance(DistanceUnit.CM);
    }

    public void resetLiftEncoders () {
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startTapeMeasure () {
        tapeMeasureServo.setPosition(1);
    }

    public void stopTapeMeasure () {
        tapeMeasureServo.setPosition(0);
    }
}

class Constants {
    final static DcMotor.RunMode AUTO_RUN_MODE = DcMotor.RunMode.RUN_USING_ENCODER, TELEOP_RUN_MODE = DcMotor.RunMode.RUN_USING_ENCODER;
    final static Servo.Direction DEFAULT_SERVO_DIRECTION = Servo.Direction.FORWARD;

    //TODO Define these constants
    final static double WHEEL_DIAMETER = 0, TICKS_PER_ROTATION = 0;
    final static double INCHES_PER_ROTATION = Math.PI * WHEEL_DIAMETER;
    final static double TICKS_PER_INCH = TICKS_PER_ROTATION / INCHES_PER_ROTATION;
    final static double DEGREES_THRESHOLD = 2;

    //TODO FINISH THESE CONSTANTS
    final static double LATCHED_POSITION_1 = 0, UNLATCHED_POSITION_1 = 1, LATCHED_POSITION_2 = 1, UNLATCHED_POSITION_2 = 0;
    final static double HUNGRY_HIPPO_RETRACT_POSITION = 0, HUNGRY_HIPPO_EXTEND_POSITION = 1;
    final static double INTAKE_SERVO_OUT_POSITION = 1, INTAKE_SERVO_IN_POSITION = 0;
    final static double GRABBER_OPEN_POSITION = 0, GRABBER_CLOSE_POSITION = 1; //reversed this on 2/10!

    enum IntakeState { INTAKE, OUTTAKE, STOP }

    final static double INTAKE_POWER_FAST = 0.7, OUTTAKE_POWER_FAST = -INTAKE_POWER_FAST,
            INTAKE_POWER_SLOW = 0.45, OUTTAKE_POWER_SLOW = -INTAKE_POWER_SLOW, STOP_POWER = 0; //.7 too fast

    enum IntakeSpeed { FAST, SLOW, STOPPED }

    enum LatchSide { LEFT, RIGHT }

    final static int MIN_ENCODER_DIFFERENCE_INTAKE = 1;

    final static double SLOW_LIFT_POWER_UP = 0.3, SLOW_LIFT_POWER_DOWN = -SLOW_LIFT_POWER_UP;
}
