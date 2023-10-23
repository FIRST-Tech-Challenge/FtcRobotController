package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

public abstract class RobotOpMode extends OpMode {

    /**
     * Used in the endTime parameter in moveRobot()
     */
    public static long STOP_NEVER = Long.MAX_VALUE;
    public DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, armMotor;
    public DcMotor armExtensionMotor;
    public Servo armCatcherServo, wristServo, fingerServo;
    public float armCatcherServoPosition;
    BNO055IMU imu;
    ElapsedTime elapsedTime;
    /**
     * A initializer for FTCDashboardPackets(), which can be used to show telemetry on the dashboard
     */
    public final FTCDashboardPackets dbp = new FTCDashboardPackets();

    public int armZeroPosition = 0;


    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl_drv");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl_drv");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr_drv");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br_drv");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        //armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension");

        armCatcherServo = hardwareMap.get(Servo.class, "arm_servo");
        armCatcherServoPosition = (float) armCatcherServo.getPosition();

        //fingerServo = hardwareMap.get(Servo.class, "finger_servo");
        //wristServo = hardwareMap.get(Servo.class, "wrist_servo");

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
        } catch(Exception e) {
            dbp.createNewTelePacket();
            dbp.put("WARNING", "Error Initializing IMU: "+e.getMessage());
            dbp.send(true);
        }
        if(imu != null) {
            dbp.createNewTelePacket();
            dbp.put("INFO", "IMU Connected Successfully!");
            dbp.send(true);
        }
        BNO055IMU.Parameters paramaters = new BNO055IMU.Parameters();
        paramaters.temperatureUnit = BNO055IMU.TempUnit.CELSIUS;
        paramaters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        paramaters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(paramaters);
    }

    @Override
    public final void loop() {
        robotloop();
    }

    public abstract void robotloop();

    /**
     * Sets the power of the robot's drive motors according to the parameters
     *
     * @param axial FORWARD AND BACKWARD
     * @param lateral STRAFING, SIDE TO SIDE
     * @param yaw ROTATION
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        moveRobot(axial, lateral, yaw, STOP_NEVER);
    }

    /**
     * Sets the power of the robot's drive motors according to the parameters
     *
     * @param axial FORWARD AND BACKWARD
     * @param lateral STRAFING, SIDE TO SIDE
     * @param yaw ROTATION
     * @param endTime the nanoTime that the robot should stop doing the move action
     * @return if the nanoTime of elapsedTime has <strong>NOT</strong> exceeded endTime.
     */
    public boolean moveRobot(double axial, double lateral, double yaw, long endTime) {

        double max;
        if(elapsedTime.now(TimeUnit.NANOSECONDS) >= endTime) {
            resetDriveMotors();
            return false;
        }

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        return true;
    }

    public int initArm() {
        dbp.createNewTelePacket();
        dbp.put("Initializing", "Starting up");
        dbp.send(true);

        armMotor.setTargetPosition(270);
        armMotor.setPower(0.1);

        elapsedTime.reset();

        double startTime = 0;
        final double GRACE_PERIOD_LENGTH = 2.0f;
        final int POWER_ROC_CUTOFF = 3;
        boolean gracePeriod = true;

        int previousPosition = armMotor.getCurrentPosition();
        while (armMotor.isBusy()) {
            if (gracePeriod) {
                dbp.put("Grace Period", "Waiting...");
                if (elapsedTime.seconds() >= GRACE_PERIOD_LENGTH) {
                    dbp.put("Grace Period", "Exiting...");
                    dbp.send(true);
                    gracePeriod = false;
                    elapsedTime.reset();
                } else {
                    dbp.put("Grace Period", "Continuing...");
                    dbp.send(true);
                    continue;
                }
            }
            final double PRESENT_GRACE_PERIOD = (startTime + GRACE_PERIOD_LENGTH);
            final boolean HAS_GRACE_PERIOD_PASSED =
                    (elapsedTime.seconds() >= PRESENT_GRACE_PERIOD);

            dbp.put("Elapsed Time", String.valueOf(elapsedTime.seconds()));
            dbp.put("Start time + Grace Period Length", String.valueOf(PRESENT_GRACE_PERIOD));
            dbp.put("HAS GRACE PERIOD PASSED", HAS_GRACE_PERIOD_PASSED ? "YES" : "NO");

            if (HAS_GRACE_PERIOD_PASSED) {
                dbp.put("Init Testing", "Checking if ROC hit cutoff...");
                final int CURRENT_ROC = (armMotor.getCurrentPosition() - previousPosition);
                dbp.put("CURRENT ROC", String.valueOf(CURRENT_ROC));
                final boolean CUTOFF_REACHED = (CURRENT_ROC < POWER_ROC_CUTOFF);
                dbp.put("CUTOFF REACHED", CUTOFF_REACHED ? "YES" : "NO");
                if (CUTOFF_REACHED)  {
                    dbp.send(false);
                    armMotor.setPower(0);
                    return armMotor.getCurrentPosition();
                }
                dbp.send(true);
                previousPosition = armMotor.getCurrentPosition();
            }
            dbp.send(true);
            //startTime = elapsedTime.seconds();
        }
        return 0;
    }

    /**
     * Moves a specified servo to the inputted degrees
     * @param servo The servo to move
     * @param position The position, in degrees, to move the servo
     */
    public void moveServo(Servo servo, int position) {
        dbp.createNewTelePacket();

        if (position > 180) {
            position = 180;
        }
        if (position < 0) {
            position = 0;
        }

        final float POSITION_TO_ANALOG = ( (float) position)/180;

        dbp.put("Move Servo", String.format(Locale.ENGLISH,
                "Moving servo to: %f", POSITION_TO_ANALOG));
        servo.setPosition(POSITION_TO_ANALOG);
        dbp.send(false);

        armCatcherServoPosition = POSITION_TO_ANALOG;
    }

    public void moveServo(Servo servo, float position) {
        dbp.createNewTelePacket();

        if (position > 1) {
            position = 1;
        }
        if (position < 0) {
            position = 0;
        }

        dbp.put("Move Servo", String.format(Locale.ENGLISH,
                "Moving servo to: %f", position));
        servo.setPosition(position);
        dbp.send(false);

        armCatcherServoPosition = position;
    }

    public void moveArm(float power, int position) {
        dbp.createNewTelePacket();
        armMotor.setTargetPosition(position);

        armMotor.setPower(power);

        while (armMotor.isBusy()) {
            dbp.put("Waiting", "...");
            dbp.send(true);
        }
        armMotor.setPower(0);
    }

    public void resetDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void gamePadMoveRobot() {
        if(gamepad1 == null) {
            return;
        }

        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        // sets the power of the motors accordingly
        moveRobot(axial, lateral, yaw, Long.MAX_VALUE);
    }

    public enum ArmServoPosition {
        OPEN(0),
        CLOSED(180);

        final int position;

        ArmServoPosition(int position) {
            this.position = position;
        }

        public int getPosition() {
            return position;
        }

    }
}
