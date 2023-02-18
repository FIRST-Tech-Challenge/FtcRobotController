package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="FSM Example2")
public class StatemachineTest extends OpMode {

    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // For figuring circumference
    static final double clicksPerInch = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double fast = 0.6; // Fast speed
    private double medium = 0.3; // Medium speed
    private double slow = 0.1; // Slow speed
    private double clicksPerDeg = clicksPerInch / 4.99; // empirically measured

    private final double MAX_POWER = 0.75;
    private final double MAX_HEIGHT = 90;
    private final double LIFT_POWER_INCREMENT = 0.05;
    // when moving to a target height with the lift, this is how much flexibility the lift has
    // in reaching the target height [cm]
    private final double WIGGLE_ROOM = 1.5;

    // target heights [cm]
    private final double LOW_HEIGHT = 41.0;
    private final double MED_HEIGHT = 65.0;
    private final double HIGH_HEIGHT = 86.4;

    private boolean open = true;
    private boolean close = false;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    private DcMotor liftmotor;
    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;
    private Servo claw;
    Servo rightServo;
    Servo leftServo;
    double servoPosition = 0.0;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DistanceSensor sensorRange;
    private DigitalChannel sensorTouch;
    private  DigitalChannel limitSwitch = null;


    private ElapsedTime runtime = new ElapsedTime();

    final double DUMP_IDLE = -0.5;
    final double DUMP_DEPOSIT = 0.7;
    final double DUMP_TIME = .5;
    final int LIFT_LOW = 0;
    final int LIFT_HIGH = 100;
    double liftPower = 0.0;
    double targetheight = 0.0;

    public enum LiftState {
        START,
        LIFTING,
        LIFT_STOP,
        LIFT_SET_LOW,
        LIFTING_LOW,

        WAIT_LIFT,
        WAIT,

    }

    public enum Drivestate {
        FORWARD,
        WAIT_DRIVE
    }

    LiftState liftState = LiftState.START;
    Drivestate drivestate = Drivestate.FORWARD;


    public void init() {
        runtime.reset();
        liftMotor1 = hardwareMap.get(DcMotor.class, "LiftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);


        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        sensorTouch = hardwareMap.get(DigitalChannel.class, "sensor_touch");
        sensorTouch.setMode(DigitalChannel.Mode.INPUT);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");


    }

    public void loop() {
       /* switch (drivestate) {
            case FORWARD:
                moveForward(24,medium);
                drivestate = drivestate.WAIT_DRIVE;
        }
        break;
        case
            }*/



       /* switch (drivestate) {
            case FORWARD:
            moveForward(24,medium);
            break;}*/
        //set target to med
        targetheight=MED_HEIGHT;
        switch (liftState){
            case START:

                    liftState = liftState.LIFTING;

                break;
            case LIFTING:
                //Move lift to target position
                MoveLift(targetheight);
                //One lift reaches target height, change state to LIFT_STOP
                if (sensorRange.getDistance(DistanceUnit.CM) >= targetheight- WIGGLE_ROOM) {
                    liftState = liftState.LIFT_SET_LOW;
                    Wait(5.0);

                }
                break;
            case LIFT_SET_LOW:
               targetheight=LOW_HEIGHT;
               liftState=liftState.LIFTING_LOW;
               break;
            case LIFTING_LOW:
                MoveLift(targetheight);
                if(sensorRange.getDistance(DistanceUnit.CM) >= targetheight- WIGGLE_ROOM) {
                    liftState = liftState.LIFT_STOP;
                }
                break;
            case LIFT_STOP:
                dualLift(0.0);
                break;


        }
        //moveForward(24, medium);


        telemetry.addData("Lift Motors", "%5.2f", liftPower);
        telemetry.addData("Set Height","%5.2f", targetheight);
        telemetry.addData("Current State", liftState.toString());
        telemetry.addData("Current State",liftState.name());
        telemetry.addData("Current Height","%5.2f", sensorRange.getDistance(DistanceUnit.CM));
        telemetry.update();




    }



    public void dualPosition(int position) {

        liftMotor1.setTargetPosition(position);
        liftMotor2.setTargetPosition(position);
    }

    public void dualPower(int Power) {

        liftMotor1.setPower(Power);
        liftMotor2.setPower(Power);
    }

    public void dualRun(int Run) {

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void MoveLift(double targetHeight) {


        if (sensorRange.getDistance(DistanceUnit.CM) < targetHeight - WIGGLE_ROOM) {

            liftPower = RampUpLiftPower(liftPower);
            dualLift(liftPower);}

        else if (sensorRange.getDistance(DistanceUnit.CM) > targetHeight + WIGGLE_ROOM ){
            liftPower = RampDownLiftPower(liftPower);
            dualLift(liftPower);}
       /* else if (!limitSwitch.getState() && liftMotor1.getPower() > 0 ); {
            dualLift(0.0);
        }*/
         else {
            liftPower = 0.0;
            dualLift(liftPower);

        }


    }



    public double RampUpLiftPower(double liftPower) {

        if (liftPower < MAX_POWER) {
            return liftPower + LIFT_POWER_INCREMENT;
        }
        return liftPower;
    }

    public double RampDownLiftPower(double liftPower) {

        if (liftPower > -MAX_POWER) {
            return liftPower - LIFT_POWER_INCREMENT;
        }
        return liftPower;
    }

    public void dualLift(double power) {

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }


    private void moveForward(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += (int) (howMuch * clicksPerInch);
        rfPos += (int) (howMuch * clicksPerInch);
        lrPos += (int) (howMuch * clicksPerInch);
        rrPos += (int) (howMuch * clicksPerInch);

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d :%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }


        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void strafe(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += (int) (howMuch * clicksPerInch);
        rfPos -= (int) (howMuch * clicksPerInch);
        lrPos -= (int) (howMuch * clicksPerInch);
        rrPos += (int) (howMuch * clicksPerInch);

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d :%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void moveWholeBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (24 * clicksPerInch);
            rfPos += (int) (24 * clicksPerInch);
            lrPos += (int) (24 * clicksPerInch);
            rrPos += (int) (24 * clicksPerInch);
        }
        else if (direction == "backward") {
            lfPos += (int) (-24 * clicksPerInch);
            rfPos += (int) (-24 * clicksPerInch);
            lrPos += (int) (-24 * clicksPerInch);
            rrPos += (int) (-24 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d :%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }


        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void moveHalfBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (12 * clicksPerInch);
            rfPos += (int) (12 * clicksPerInch);
            lrPos += (int) (12 * clicksPerInch);
            rrPos += (int) (12 * clicksPerInch);
        }
        else if (direction == "backward") {
            lfPos += (int) (-12 * clicksPerInch);
            rfPos += (int) (-12 * clicksPerInch);
            lrPos += (int) (-12 * clicksPerInch);
            rrPos += (int) (-12 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d :%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void strafeWholeBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (24 * clicksPerInch);
            rfPos -= (int) (24 * clicksPerInch);
            lrPos -= (int) (24 * clicksPerInch);
            rrPos += (int) (24 * clicksPerInch);
        }
        else if (direction == "backward") {
            lfPos += (int) (-24 * clicksPerInch);
            rfPos -= (int) (-24 * clicksPerInch);
            lrPos -= (int) (-24 * clicksPerInch);
            rrPos += (int) (-24 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d :%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }


        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void strafeHalfBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (12 * clicksPerInch);
            rfPos -= (int) (12 * clicksPerInch);
            lrPos -= (int) (12 * clicksPerInch);
            rrPos += (int) (12 * clicksPerInch);
        }
        else if (direction == "backward") {
            lfPos += (int) (-12 * clicksPerInch);
            rfPos -= (int) (-12 * clicksPerInch);
            lrPos -= (int) (-12 * clicksPerInch);
            rrPos += (int) (-12 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d :%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }


        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void gripper(boolean toggle){
        if (toggle) {
            rightServo.setPosition(1.0);
            leftServo.setPosition(0.0);
        }
        else {
            rightServo.setPosition(0.0);
            leftServo.setPosition(1.0);
        }
    }

    private void turnClockwise(int whatAngle, double speed) {
        // "whatAngle" is in degrees. A negative whatAngle turns counterclockwise.

        // Fetch motor positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lrPos += whatAngle * clicksPerDeg;
        rrPos -= whatAngle * clicksPerDeg;

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void Wait(double seconds) {
        runtime.reset();
        while (runtime.time() < seconds) {

        }
    }
}

