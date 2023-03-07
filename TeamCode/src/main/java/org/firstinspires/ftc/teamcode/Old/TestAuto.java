//package org.firstinspires.ftc.teamcode.Old;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.Auto.VisionRight;
//
//
//@Autonomous(name="TestAuto")
//@Disabled
//public class TestAuto extends LinearOpMode {
//
//    DcMotor frontleft;
//    DcMotor frontright;
//    DcMotor backleft;
//    DcMotor backright;
//    DcMotor lift;
//    DcMotor rightgrabber;
//    DcMotor leftgrabber;
//
//
//    ModernRoboticsI2cGyro gyro;
//    ModernRoboticsI2cRangeSensor rangebrothers;
//
//
//
//    //28 * 20 / (2ppi * 4.125)
//    Double width = 16.0; //inches
//    Integer cpr = 13; //counts per rotation
//    Integer gearratio = 40;
//    Double diameter = 4.125;
//    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
//    Double bias = 0.8;//default 0.8
//    Double meccyBias = 0.9;//change to adjust only strafing movement
//    double amountError = 2;
//
//    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
//    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
//    static final double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stable
//
//    double DRIVE_SPEED = 0.32;
//    double TURN_SPEED = 0.4;
//    double DRIVE_ANGLE = 0;
//
//    final int LEVEL_ZERO              = 45;
//    final int LEVEL_ONE               = 1700;
//    final int LEVEL_TWO               = 3870;
//    final int LEVEL_THREE             = 5000;
//    int LEVEL_CAP               = 5500;
//    int ACCEPTABLE_ERROR        = 0;
//    int TELEOP_ACCEPTABLE_ERROR = 30;
//
//
//    double   MAX_ACCEPTABLE_ERROR = 10;
//
//
//    Double conversion = cpi * bias;
//    Boolean exit = false;
//
//    BNO055IMU imu;
//    Orientation angles;
//    Acceleration gravity;
//
//    public void runOpMode() throws InterruptedException {
//
//        //initGyro();
//
//        frontleft = hardwareMap.dcMotor.get("frontleft");
//        frontright = hardwareMap.dcMotor.get("frontright");
//        backleft = hardwareMap.dcMotor.get("backleft");
//        backright = hardwareMap.dcMotor.get("backright");
//        lift = hardwareMap.get(DcMotor.class, "lift");
//        leftgrabber = hardwareMap.get(DcMotor.class, "leftgrabber");
//        rightgrabber = hardwareMap.get(DcMotor.class, "rightgrabber");
//        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "Gyro");
//        rangebrothers = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"rangebrothers");
//
//        //rangebrothers.initialize();
//
//
//
//        backleft.setDirection(DcMotor.Direction.FORWARD);
//        frontleft.setDirection(DcMotor.Direction.FORWARD);
//        backright.setDirection(DcMotor.Direction.REVERSE);
//        frontright.setDirection(DcMotor.Direction.REVERSE);
//        lift.setDirection(DcMotor.Direction.FORWARD);
//        leftgrabber.setDirection(DcMotor.Direction.FORWARD);
//        rightgrabber.setDirection(DcMotor.Direction.FORWARD);
//
//
//
//        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftgrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightgrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftgrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightgrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftgrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightgrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        backleft.setPower(0);
//        frontleft.setPower(0);
//        backright.setPower(0);
//        frontright.setPower(0);
//        lift.setPower(0);
//        leftgrabber.setPower(0);
//        rightgrabber.setPower(0);
//
//        VisionRight detector = new VisionRight(this);
//
//
//        telemetry.addLine("Start Gyro");
//        telemetry.update();
//        gyro.calibrate();
//        while (gyro.isCalibrating()) ;
//        telemetry.addLine("Gyro Calibrated");
//        telemetry.addData("Angle: ", gyro.getIntegratedZValue());
//        telemetry.update();
//
//
//        waitForStart();
//
//
//        telemetry.addData("TRY ",1);
//        telemetry.addData("Range Value: ", rangebrothers.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Target: ", 1);
//        telemetry.update();
//
//        //neg strafe = left strafing
//
//        drivebackrightandfrontleft(-20,.4);
//
//
//
//
//
//    }
//
//    public void drivebackrightandfrontleft(double inches, double speed) {
//
//        //
//        int move = (int) (Math.round(inches * cpi * meccyBias));
//        //
//        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
//        //frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
//        //backright.setTargetPosition(backright.getCurrentPosition() + move);
//        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
//        //
//        //frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //
//        //frontleft.setPower(speed);
//        backleft.setPower(speed);
//        frontright.setPower(speed);
//        //backright.setPower(speed);
//        //
//        while ( frontright.isBusy() && backleft.isBusy()) {
//        }
//        frontright.setPower(0);
//        //frontleft.setPower(0);
//        //backright.setPower(0);
//        backleft.setPower(0);
//        return;
//
//    }
//
//    private boolean rangeCheck(ModernRoboticsI2cRangeSensor range_sensor, double desired_distance, LinearOpMode opMode){
//        final int TRIES = 3;
//        for (int i = 0; i < TRIES; i++){
//            if (Math.abs(range_sensor.getDistance(DistanceUnit.INCH) - desired_distance) < MAX_ACCEPTABLE_ERROR){
//                return true;
//            }
//            opMode.telemetry.addData("TRY ",i);
//            opMode.telemetry.addData("Range Value: ", range_sensor.getDistance(DistanceUnit.INCH));
//            opMode.telemetry.addData("Target: ", desired_distance);
//            opMode.telemetry.update();
//            try {
//                Thread.sleep(1500);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//        return false;
//    }
//
//
//
//    public void rangeDrive (double speed, double backDistance, double sideDistance, LinearOpMode opmode) {
//
//        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//
//        if (backDistance != -1) {
//            while (rangebrothers.getDistance(DistanceUnit.INCH) < backDistance) {
//                if (Math.abs(rangebrothers.getDistance(DistanceUnit.INCH) - backDistance) > MAX_ACCEPTABLE_ERROR) {
//                    if (!rangeCheck(rangebrothers, backDistance, opmode)) {
//                        break;
//                    }
//                }
//                backleft.setPower(-speed);
//                backright.setPower(-speed);
//                frontleft.setPower(-speed);
//                frontright.setPower(-speed);
//
//                opmode.telemetry.addData("Sensor Back Distance: ", rangebrothers.getDistance(DistanceUnit.INCH));
//                opmode.telemetry.addData("Target Back Distance: ", backDistance);
//                opmode.telemetry.addLine("Moving Backwards");
//                opmode.telemetry.update();
//            }
//            while (rangebrothers.getDistance(DistanceUnit.INCH) > backDistance) {
//                if (Math.abs(rangebrothers.getDistance(DistanceUnit.INCH) - backDistance) > MAX_ACCEPTABLE_ERROR) {
//                    if (!rangeCheck(rangebrothers, backDistance, opmode)) {
//                        break;
//                    }
//                }
//                backleft.setPower(speed);
//                backright.setPower(speed);
//                frontleft.setPower(speed);
//                frontright.setPower(speed);
//
//                opmode.telemetry.addData("Sensor Back Distance: ", rangebrothers.getDistance(DistanceUnit.INCH));
//                opmode.telemetry.addData("Target Back Distance: ", backDistance);
//                opmode.telemetry.addLine("Moving Forwards");
//                opmode.telemetry.update();
//            }
//        }
//    }
//}
