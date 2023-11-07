package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

public abstract class RobotOpMode extends OpMode {

    /**
     * Used in the endTime parameter in moveRobot()
     */
    public static long STOP_NEVER = Long.MAX_VALUE;
    public static float MIN_POWER = 0;
    public static float MAX_POWER = 1;

    public DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    public DcMotor armMotor, armExtensionMotor;
    public Servo armCatcherServo, wristServo, fingerServo;
    @Deprecated
    public float armCatcherServoPosition;
    public double wristServoPosition;
    BNO055IMU imu;
    ElapsedTime elapsedTime;
    /**
     * A initializer for FTCDashboardPackets(), which can be used to show telemetry on the dashboard
     */
    public final FTCDashboardPackets dbp = new FTCDashboardPackets();
    public int armZeroPosition = 0;
    private HandState targetHandState;

    @Override
    public void init() {
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "fl_drv");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "fr_drv");
            leftBackDrive = hardwareMap.get(DcMotor.class, "bl_drv");
            rightBackDrive = hardwareMap.get(DcMotor.class, "br_drv");
        } catch(Exception e) {
            createTelemetryPacket();
            log("FATAL ERROR", e.getMessage());
            sendTelemetryPacket(true);
            requestOpModeStop();
            return;
        }

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        createTelemetryPacket();

        try {
            armMotor = hardwareMap.get(DcMotor.class, "arm");
            armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension");

            armCatcherServo = hardwareMap.get(Servo.class, "arm_servo");
            //armCatcherServoPosition = (float) armCatcherServo.getPosition();

            fingerServo = hardwareMap.get(Servo.class, "finger_servo");
            wristServo = hardwareMap.get(Servo.class, "wrist_servo");
            wristServoPosition = wristServo.getPosition();
            setTargetHandState(HandState.RELEASE);
        } catch(Exception e) {
            log(ERROR, "Error initializing arm and wrist motors: "+e.getMessage());
        }

        try {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            log(ERROR, "Error initializing drive motors: "+e.getMessage());
        }

        try {
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setTargetPosition(0);
            //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armExtensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            //initArm();
        } catch(Exception e) {
            log(ERROR, "Error initializing arm motor: "+e.getMessage());
        }

        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.temperatureUnit = BNO055IMU.TempUnit.CELSIUS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        } catch(Exception e) {
            log(WARNING, "Error initializing IMU: "+e.getMessage());
        }
        if(imu != null) {
            log(INFO, "IMU connected successfully!");
        }
        sendTelemetryPacket(true);
    }

    @Override
    public final void loop() {
        robotLoop();
        telemetry.update();
    }

    public abstract void robotLoop();

    @Override
    public void stop() {
        resetDriveMotors();
        resetArmMotors();
    }

    /**
     * Sets the power of the robot's drive motors according to the parameters
     *
     * @param axial FORWARD AND BACKWARD
     * @param lateral STRAFING, SIDE TO SIDE
     * @param yaw ROTATION
     */
    public void linearMoveRobot(double axial, double lateral, double yaw) {
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
        createTelemetryPacket();
        log("Initializing", "Starting up...");
        sendTelemetryPacket(true);

        armMotor.setTargetPosition(270);
        armMotor.setPower(0.1);

        elapsedTime.reset();

        double startTime = 0;
        final double GRACE_PERIOD_LENGTH = 2.0f;
        final int POWER_ROC_CUTOFF = 3;
        boolean gracePeriod = true;

        createTelemetryPacket();
        int previousPosition = armMotor.getCurrentPosition();
        while (armMotor.isBusy()) {
            if (gracePeriod) {
                log("Grace Period", "Waiting...");
                if (elapsedTime.seconds() >= GRACE_PERIOD_LENGTH) {
                    log("Grace Period", "Exiting...");
                    sendTelemetryPacket(true);
                    gracePeriod = false;
                    elapsedTime.reset();
                } else {
                    log("Grace Period", "Continuing...");
                    sendTelemetryPacket(true);
                    continue;
                }
            }
            final double PRESENT_GRACE_PERIOD = (startTime + GRACE_PERIOD_LENGTH);
            final boolean HAS_GRACE_PERIOD_PASSED =
                    (elapsedTime.seconds() >= PRESENT_GRACE_PERIOD);

            log("Elapsed Time", String.valueOf(elapsedTime.seconds()));
            log("Start time + Grace Period Length", String.valueOf(PRESENT_GRACE_PERIOD));
            log("HAS GRACE PERIOD PASSED", HAS_GRACE_PERIOD_PASSED ? "YES" : "NO");

            if (HAS_GRACE_PERIOD_PASSED) {
                log("Init Testing", "Checking if ROC hit cutoff...");
                final int CURRENT_ROC = (armMotor.getCurrentPosition() - previousPosition);
                log("CURRENT ROC", String.valueOf(CURRENT_ROC));
                final boolean CUTOFF_REACHED = (CURRENT_ROC < POWER_ROC_CUTOFF);
                log("CUTOFF REACHED", CUTOFF_REACHED ? "YES" : "NO");
                if (CUTOFF_REACHED)  {
                    sendTelemetryPacket(false);
                    armMotor.setPower(MIN_POWER);
                    return armMotor.getCurrentPosition();
                }
                sendTelemetryPacket(true);
                previousPosition = armMotor.getCurrentPosition();
            }
            sendTelemetryPacket(true);
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
        createTelemetryPacket();

        if (position > 180) {
            position = 180;
        }
        if (position < 0) {
            position = 0;
        }

        final float POSITION_TO_ANALOG = ( (float) position)/180;


        log("Move Servo", String.format(Locale.ENGLISH,
                "Moving servo to: %f", POSITION_TO_ANALOG));
        try {
            servo.setPosition(POSITION_TO_ANALOG);
        } catch (Exception e) {
            dbp.put("Error", e.getMessage());
        }
        sendTelemetryPacket(false);
    }

    public void moveServo(Servo servo, float position) {
        createTelemetryPacket();

        if (position > 1) {
            position = 1;
        }
        if (position < 0) {
            position = 0;
        }

        log("Move Servo", String.format(Locale.ENGLISH,
                "Moving servo to: %f", position));
        try {
            servo.setPosition(position);
        } catch (Exception e) {
            dbp.put("Error", e.getMessage());
        }

        sendTelemetryPacket(false);
    }

    public void moveArm(float power, int position) {
        createTelemetryPacket();
        armMotor.setTargetPosition(position);

        armMotor.setPower(power);

        // FIXME: This will freeze the entire program whenever the arm moves. Implement a fix that works in a LinearOpMode or clarify this?
        while (armMotor.isBusy()) {
            log("Waiting", "...");
            sendTelemetryPacket(true);
        }
        armMotor.setPower(MIN_POWER);
    }

    /**
     * Moves the arm according to the parameters. This should be used in any
     * {@link com.qualcomm.robotcore.eventloop.opmode.LinearOpMode} that extends {@link RobotOpMode}
     *
     * @param power The power of the arm motor
     * @param position The target position of the arm motor
     * @return if the arm motor has successfully reached the position
     */
    public boolean linearMoveArm(float power, ArmServoPosition position) {
        armMotor.setTargetPosition(getArmPosition(position));
        armMotor.setPower(power);
        boolean busy = armMotor.isBusy();
        // FIXME: I am unsure of how the isBusy method works. Make sure it doesn't return true when setting position.
        // It may calculate how busy the motor is based on velocity OR it may determine it by position.
        // For the first scenario, manually check the position of the robot and apply changes accordingly.
        // For the second scenario, this should work perfectly fine
        if(!busy) {
            armMotor.setPower(MIN_POWER);
        }
        return !busy;
    }

    public static enum HandState {
        GRAB {
            @Override
            public boolean handStateMet(RobotOpMode mode) {
                // TODO: check the position of the hand and return the value accordingly
                return false;
            }

            @Override
            public void onTargetStateSet(RobotOpMode mode) {
                // TODO: set the target position of the hand motors accordingly
            }
        },
        RELEASE {
            @Override
            public boolean handStateMet(RobotOpMode mode) {
                // TODO: check the position of the hand and return the value accordingly
                return false;
            }

            @Override
            public void onTargetStateSet(RobotOpMode mode) {
                // TODO: set the target position of the hand motors accordingly
            }
        };

        public abstract boolean handStateMet(RobotOpMode mode);
        public abstract void onTargetStateSet(RobotOpMode mode);
    }

    public void setTargetHandState(HandState targetHandState) {
        this.targetHandState = targetHandState;
        this.targetHandState.onTargetStateSet(this);
    }
    public boolean isHandActionComplete() {
        return targetHandState.handStateMet(this);
    }

    /**
     * Sets the power of the wheel motors to zero.
     */
    public void resetDriveMotors() {
        leftFrontDrive.setPower(MIN_POWER);
        rightFrontDrive.setPower(MIN_POWER);
        leftBackDrive.setPower(MIN_POWER);
        rightBackDrive.setPower(MIN_POWER);
    }

    /**
     * Sets the power of the arm motors to zero
     */
    public void resetArmMotors() {
        try {
            armMotor.setPower(MIN_POWER);
            armExtensionMotor.setPower(MIN_POWER);
        } catch(NullPointerException e) {
            log(WARNING, e.getMessage());
        }
    }

    /**
     * Navigates the robot according to the inputs of Game-pad ONE
     */
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

    /**
     * Obtains the position of the arm relative to the generated zero position
     * @param position The position of the arm servo
     * @return The target position variable that can be used by the arm motor.
     */
    public int getArmPosition(ArmServoPosition position) {
        return position.getPosition()-armZeroPosition;
    }

    public static final String ERROR = "ERROR";
    public static final String WARNING = "WARNING";
    public static final String INFO = "INFO";

    public void createTelemetryPacket() {
        dbp.createNewTelePacket();
    }
    public void sendTelemetryPacket(boolean reinitializePacket) {
        dbp.send(reinitializePacket);
    }
    public void log(String header, String message) {
        dbp.put(header, message);
        telemetry.addData(header, message);
    }

    public enum ArmServoPosition {
        OPEN(-45),
        CLOSED(45);

        final int position;

        ArmServoPosition(int position) {
            this.position = position;
        }

        public int getPosition() {
            return position;
        }

    }
}
