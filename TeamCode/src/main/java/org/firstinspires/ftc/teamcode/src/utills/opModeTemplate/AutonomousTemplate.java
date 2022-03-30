package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.NavigationalDrivetrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.IMUOdometry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.IMU;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.intake.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;

/**
 * A template for Autonomous OpModes, allows for easy initialization
 */
@Disabled
public abstract class AutonomousTemplate extends GenericOpModeTemplate {


    /**
     * Provides methods to use Odometry and associated drive methods
     */
    protected NavigationalDrivetrain driveSystem;


    /**
     * Provides Localization Methods, Also held by NavigationalDrivetrain driveSystem
     */
    protected LocalizationAlgorithm gps;

    /**
     * Allows the control of the intake
     */
    protected ContinuousIntake intake;


    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if the OpMode is stopped during function execution
     */
    protected void initAll() throws InterruptedException {

        this.initVoltageSensor();

        DcMotor front_right;
        DcMotor front_left;
        DcMotor back_left;
        DcMotor back_right;

        //Init the drive Motors
        {
            front_right = hardwareMap.dcMotor.get(frontRightName);
            front_left = hardwareMap.dcMotor.get(frontLeftName);
            back_left = hardwareMap.dcMotor.get(backLeftName);
            back_right = hardwareMap.dcMotor.get(backRightName);

            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            back_right.setDirection(DcMotorSimple.Direction.REVERSE);
            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        IMU imu1;

        // Init IMU for Odometry
        {
            checkStop();
            imu1 = new IMU(hardwareMap, IMUName);
            checkStop();
        }

        DcMotor frontIntakeMotor;
        DcMotor backIntakeMotor;
        DcMotor horizontalEncoder;

        //Init motors for intakes and the encoders for Odometry
        {
            frontIntakeMotor = hardwareMap.dcMotor.get(GenericOpModeTemplate.frontIntakeMotorName);
            backIntakeMotor = hardwareMap.dcMotor.get(GenericOpModeTemplate.backIntakeMotorName);
            horizontalEncoder = hardwareMap.dcMotor.get(GenericOpModeTemplate.horizontalOdometryName);

            frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            horizontalEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        //Assemble Odometry from intake motors and horizontal encoder
        {
            IMUOdometry o = new IMUOdometry(backIntakeMotor, frontIntakeMotor, horizontalEncoder, imu1.getImu(), 25, this::opModeIsActive, this::isStopRequested);
            //o.reverseRightEncoder();

            o.start();
            gps = o;
        }

        //Assemble Drive system from Odometry and Drive Motors
        driveSystem = new NavigationalDrivetrain(front_right, front_left, back_right, back_left, telemetry, gps, this::isStopRequested, this::opModeIsActive, voltageSensor);

        // Init Intake from intake motors
        intake = new ContinuousIntake(backIntakeMotor, frontIntakeMotor);


        super.initAll();
        podServos.lower();
        slide.setTargetLevel(HeightLevel.Down);

        telemetry.addData("Default Initialization: ", "Finished");
        telemetry.update();
        checkStop();

    }

    /**
     * Initializes all fields provided by this class but the wheels are not in break mode
     *
     * @throws InterruptedException Throws if the OpMode is stopped during function execution
     */
    protected void initAllWithWheelDrift() throws InterruptedException {
        this.initVoltageSensor();

        DcMotor front_right;
        DcMotor front_left;
        DcMotor back_left;
        DcMotor back_right;

        //Init the drive Motors
        {
            front_right = hardwareMap.dcMotor.get(frontRightName);
            front_left = hardwareMap.dcMotor.get(frontLeftName);
            back_left = hardwareMap.dcMotor.get(backLeftName);
            back_right = hardwareMap.dcMotor.get(backRightName);

            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


            back_right.setDirection(DcMotorSimple.Direction.REVERSE);
            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        IMU imu1;

        // Init IMU for Odometry
        {
            checkStop();
            imu1 = new IMU(hardwareMap, IMUName);
            checkStop();
        }

        DcMotor frontIntakeMotor;
        DcMotor backIntakeMotor;
        DcMotor horizontalEncoder;

        //Init motors for intakes and the encoders for Odometry
        {
            frontIntakeMotor = hardwareMap.dcMotor.get(GenericOpModeTemplate.frontIntakeMotorName);
            backIntakeMotor = hardwareMap.dcMotor.get(GenericOpModeTemplate.backIntakeMotorName);
            horizontalEncoder = hardwareMap.dcMotor.get(GenericOpModeTemplate.horizontalOdometryName);

            frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            horizontalEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        //Assemble Odometry from intake motors and horizontal encoder
        {
            IMUOdometry o = new IMUOdometry(backIntakeMotor, frontIntakeMotor, horizontalEncoder, imu1.getImu(), 25, this::opModeIsActive, this::isStopRequested);
            o.start();
            gps = o;
        }

        //Assemble Drive system from Odometry and Drive Motors
        driveSystem = new NavigationalDrivetrain(front_right, front_left, back_right, back_left, telemetry, gps, this::isStopRequested, this::opModeIsActive, voltageSensor);

        // Init Intake from intake motors
        intake = new ContinuousIntake(backIntakeMotor, frontIntakeMotor);


        super.initAll();
        podServos.lower();
        slide.setTargetLevel(HeightLevel.Down);

        telemetry.addData("Default Initialization: ", "Finished");
        telemetry.update();
        checkStop();
    }

    public void dropOffFreight() throws InterruptedException {
        dropOffFreight(BarcodePositions.Right);
    }

    public void dropOffFreight(BarcodePositions Pos, double tuningFactor) throws InterruptedException {
        switch (Pos) {
            case NotSeen:
            case Right:
                // got to the top level when right
                slide.setTargetLevel(HeightLevel.TopLevel);
                break;

            case Center:
                slide.setTargetLevel(HeightLevel.MiddleLevel);
                break;

            case Left:
                // go to bottom when left
                slide.setTargetLevel(HeightLevel.BottomLevel);
                break;

        }

        //Waits for the slide to get to it's position

        double[] initialPos = {gps.getX(), gps.getY()};
        slide.setMotorPower(1);

        ElapsedTime slideStuckTimer = new ElapsedTime();
        while (!slide.isAtPosition()) {
            if (slideStuckTimer.seconds() > 8) {
                RobotLog.addGlobalWarningMessage("Slide Was Stuck. Dropping on lowest level");
                slide.setTargetLevel(HeightLevel.BottomLevel);
                break;
            }
            Thread.yield();
            if (Thread.currentThread().isInterrupted()){
                throw new InterruptedException();
            }
        }

        //Strafes forward while the distance from the wall is less than 24 in
        driveSystem.strafeAtAngle(180, 0.5);

        double currentWallDistance;
        do {
            if (MiscUtils.distance(initialPos[0], initialPos[1], gps.getX(), gps.getY()) > 24) {
                break;
            }
            currentWallDistance = Math.abs((frontDistanceSensor.getDistance(DistanceUnit.INCH)) * Math.cos(Math.toRadians(gps.getRot() - 270)));
        } while (currentWallDistance < (23 + tuningFactor) && opModeIsActive() && !isStopRequested());


        driveSystem.halt();
        outtake.setServoOpen();
        Thread.sleep(750);
        driveSystem.move(0, 5, 1, new DistanceTimeoutWarning(500));
        outtake.setServoClosed();
        slide.setTargetLevel(HeightLevel.Down);
        slide.setMotorPower(0);

    }

    public void dropOffFreight(BarcodePositions pos) throws InterruptedException {
        dropOffFreight(pos, 0);
    }

}
