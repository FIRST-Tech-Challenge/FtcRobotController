package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Robot: Red Right MarkStrike+Park", group = "Robot")
//@Disabled
public class EricTEST1_RR_MS_Park extends LinearOpMode {
    //Apriltag ID variable
    private BNO055IMU imu = null;      // Control/Expansion Hub IMU ==>this is for turning

    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;
    static final double HEADING_THRESHOLD = 0.5;    // How close must the heading get to the target before moving to next step.

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearRight = null;

    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //double maxTurnSpeed;
    //static final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    //static final double TURN_SPEED = 0.2;     // Max Turn speed to limit turn rate
    /* Declare OpMode members. */
    //private DcMotor         leftDrive   = null;
    //private DcMotor         rightDrive  = null;
    //Apriltag ID variable
    public int DESIRED_TAG_ID = 0;


    //distance sensor

    //    private DistanceSensor sensorRange;
//    // you can also cast this to a Rev2mDistanceSensor if you want to use added
//    // methods associated with the Rev2mDistanceSensor class.
//    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
//************add the drive to apriltag variable********************
//    public class org.firstinspires.ftc.teamcode.MM_RearAutoDrive_rearCamera extends LinearOpMode {
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 0.5; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

//        private DcMotor frontLeft = null;  //  Used to control the left front drive wheel
//        private DcMotor frontRight = null;  //  Used to control the right front drive wheel
//        private DcMotor rearLeft = null;  //  Used to control the left back drive wheel
//        private DcMotor rearRight = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    //private static final int DESIRED_TAG_ID = 5;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    //*************************************************************
    //static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle was 0.01
    //    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotatiovf nal position was set to 1
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.3;     // Max driving speed for better distance accuracy.

    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    private double turnSpeed = 0;

    private DistanceSensor sensorDistance;
    private DistanceSensor sensorDistanceLeft;
    private DistanceSensor sensorDistanceRight;

    //Servo servo;

    //double position = MIN_POS;//(MAX_POS - MIN_POS) / 2; // Start at halfway position
    //boolean rampUp = true;
//    private static final String TFOD_MODEL_ASSET = "Blue.tflite";//"qr code.tflite";
    //private static final String[] LABELS = {
//                "green",    //zone 1
//                "pink",     //zone 2
//                "yellow"    //zone 3
//            "Sleeve 1", //zone 2
//            "Signal Sleeve 2", //zone 3
//            "Signal Sleeve 3",//zone 1

    //      };
//    private static final String VUFORIA_KEY =
//            "AfeWy0n/////AAABmTa/5ta3ykwBsVsWn2OWhesMBidCMcGKFZ1Rqcj/0vs0EF/FGAE13yJilyo2rC5+rlURFd8y9SchsLSKI0UaCCYz+ni/NM3hJcKG2n3khAj7dybnwAnvgo8j8IfDQTH+H10o5eGtutZEJSlZWbyMmkgBl2+4LXCoteyWpe0kOpm6NiNR/QF7qzBYj4YBJEwt/LqUgm0kxjAGupIKCn2FhtfeRg4DyNpXmSVs0FEvmQqEkaAnocDq8cIZ1J/UVWPqgNmdfW+6RpOrjOK/YQMHkQfoZMYZeyZDiiG28+8/+pJJUbSHc69a6cVxO1S5EYhLWMSThTkdk7aUvW5R59hyXIIflgxMQde021tOBdbBrDq1";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    // private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
//    private TFObjectDetector tfod;
    //below tensor flow variables
//    private static final String[] LABELS = {
//            "pixel"
////            "RRC",
////            "RRL",
////            "RRR"
//    };
//
//    /**
//     * The variable to store our instance of the TensorFlow Object Detection processor.
//     */
//    private TfodProcessor tfod;
//

    /**
     * The variable to store our instance of the vision portal.
     */
//    private VisionPortal visionPortal;
//    static Boolean isPropDetected;

    //above tensor flow variable*****************
    //add array list ? Maybe?
    //   public void runMotorsForTime(double y, double x, double rx, double time, double distance) {
    public void runMotorsForTime(double y, double x, double rx, double time) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - (x * 1.1) + rx) / denominator;
        double rearLeftPower = (y + (x * 1.1) + rx) / denominator;
        double frontRightPower = -(y - (x * 1.1) - rx) / denominator;
        double rearRightPower = -(y + (x * 1.1) - rx) / denominator;
        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.addData("move forward for 3 seconds", "Complete");
            telemetry.update();
        }
    }

    public void runMotorsWithTurning(double y, double x, double heading) {
        double rx = 0;//=getSteeringCorrection(heading, P_DRIVE_GAIN);
        double denominator = 0;// Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = 0;// (y + (x * 1.1) + rx) / denominator;
        double rearLeftPower = 0;//(y - (x * 1.1) + rx) / denominator;
        double frontRightPower = 0;//(y - (x * 1.1) - rx) / denominator;
        double rearRightPower = 0;//(y + (x * 1.1) - rx) / denominator;
        //turnSpeed=heading;

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            // Determine required steering to keep on heading
            rx = getSteeringCorrection(heading, P_DRIVE_GAIN);
            // Clip the speed to the maximum permitted value.
            rx = Range.clip(rx, -0.5, 0.5);

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + (x * 1.1) + rx) / denominator;
            rearLeftPower = (y - (x * 1.1) + rx) / denominator;
            frontRightPower = (y - (x * 1.1) - rx) / denominator;
            rearRightPower = (y + (x * 1.1) - rx) / denominator;
            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(rearLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
        }
        //Stop all motion;
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }


    @Override
    public void runOpMode() {
////
//           //*********************add april tag*****************************
//            boolean targetFound     = false;    // Set to true when an AprilTag target is detected
//            double  drive           = 0;        // Desired forward power/speed (-1 to +1)
//            double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
//            double  turn            = 0;        // Desired turning power/speed (-1 to +1)
//
//            // Initialize the AprilTag Detection process
//            initAprilTag();
//            if (USE_WEBCAM)
//                setManualExposure(4, 240);  // Use low exposure time to reduce motion blur
//            //was 6 and 250
//            // Wait for driver to press start
//            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
//            telemetry.addData(">", "Touch Play to start OpMode");
//            telemetry.update();
//
//            //****************************************************************


        // Reverse the right side motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");

        // Reverse left motors if you are using NeveRests
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //lift
        DcMotor liftRight = hardwareMap.dcMotor.get("liftRight");
        DcMotor liftLeft = hardwareMap.dcMotor.get("liftLeft");
        //set direction of the lift motors be different directions
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //IntakeWheel
        DcMotor IntakeWheel = hardwareMap.dcMotor.get("IntakeWheel");

        IntakeWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //servoPlacer
        Servo servoPlacer;
        double MAX_POS = 1.0;
        double MIN_POS = 0.0;
        servoPlacer = hardwareMap.get(Servo.class, "servoPlacer");

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "sensor_left");
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "sensor_right");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        //servo = hardwareMap.get(Servo.class, "GrabberServo");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at fl rl fr rr", "%7d :%7d",
                frontLeft.getCurrentPosition(),
                rearLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                rearRight.getCurrentPosition());
        telemetry.update();
        encoderDrive(0.65, 21, 21, 21, 21, 3);//tested with higher speed but then we need to reduce distance
        //encoder does not work perfectly when speed is high lower speed like 0.3 or 0.4 works better on the distance estimate

        while (opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName", sensorDistance.getDeviceName());
//            telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
//            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("deviceName", sensorDistanceLeft.getDeviceName());
//            telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
//            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorDistanceLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("deviceName", sensorDistanceRight.getDeviceName());
//            telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
//            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorDistanceRight.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
            //encoderDrive(0.65,20,20,20,20,10);//tested with higher speed but then we need to reduce distance
            //encoder does not work perfectly when speed is high lower speed like 0.3 or 0.4 works better on the distance estimate
            //encoderDrive(0.3,20,20,20,20,10);
            if (sensorDistance.getDistance(DistanceUnit.INCH) < 6) {
                DESIRED_TAG_ID = 5;
                telemetry.addData("Middle found, ID", DESIRED_TAG_ID);
                telemetry.update();
                sleep(1000);
                break;
            }
            if (sensorDistanceLeft.getDistance(DistanceUnit.INCH) < 8) {
                DESIRED_TAG_ID = 4;
                telemetry.addData("left found, ID", DESIRED_TAG_ID);
                telemetry.update();
                sleep(1000);
                break;
            }
            if (sensorDistanceRight.getDistance(DistanceUnit.INCH) < 8) {
                DESIRED_TAG_ID = 6;
                telemetry.addData("right found, ID", DESIRED_TAG_ID);
                telemetry.update();
                sleep(1000);
                break;
            } else {
                DESIRED_TAG_ID = 4;
                telemetry.addData("Not found, Default to left ID", DESIRED_TAG_ID);
                telemetry.update();
                sleep(1000);
            }
        }
//        telemetry.addData("DESIRED ID FOUND ID", DESIRED_TAG_ID);
//        telemetry.update();
//        sleep(1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

        // Turn off RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (DESIRED_TAG_ID == 5) {

            runMotorsWithTurning(0, 0, 180);
            sleep(50);
            encoderDrive(0.35, 3, 3, 3, 3, 1);
            //sleep(1000);
            //drop purple pixel here
            IntakeWheel.setPower(-0.3);
            sleep(1000);
            IntakeWheel.setPower(0);
            encoderDrive(0.35, 19, 19, 19, 19, 2.0);
            sleep(1000);
            runMotorsWithTurning(0, 0, -90);
            encoderDrive(0.35, 36, 36, 36, 36, 5);
            //drop the pixel on backstage score 3 points
            double liftspeed = 0.3;
            //lift goes up with runtime of 2 second
            liftRight.setPower(liftspeed);
            liftLeft.setPower(liftspeed);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            //place the pixel
            servoPlacer.setPosition(MIN_POS);
            sleep(1500);
            servoPlacer.setPosition(0.65 * MAX_POS);
            sleep(500);

            //lower the lift while placing
            liftspeed = 0.25;
            liftRight.setPower(-liftspeed);
            liftLeft.setPower(-liftspeed);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }


        } else if (DESIRED_TAG_ID == 6) {
//            telemetry.addData(">","running DESIRED_TAG_ID = 3");
//            telemetry.update();
//            sleep(1000);
//            if (DESIRED_TAG_ID == 6) {
            //encoderDrive(0.25, 28, 28, 28, 28, 5.0);  // S1: backward 48 Inches with 5 Sec timeout
            runMotorsWithTurning(0, 0, 90); //rotate counterclockwise 90 degree
            //sleep(1000);
            //drop pixel here
            IntakeWheel.setPower(-0.25);
            sleep(1000);
            IntakeWheel.setPower(0);
            encoderDrive(0.35, 2, 2, 2, 2, 1);
            runMotorsForTime(0, 0.3, 0, 1.6);
            runMotorsWithTurning(0, 0, -90); //rotate clockwise 90 degree
            //runMotorsForTime(0,-0.3,0,0.6);
            sleep(50);
            encoderDrive(0.65, 33, 33, 33, 33, 5);
            sleep(50);//park
            //drop the pixel on backstage score 3 points
            double liftspeed = 0.3;
            //lift goes up with runtime of 2 second
            liftRight.setPower(liftspeed);
            liftLeft.setPower(liftspeed);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            //place the pixel
            servoPlacer.setPosition(MIN_POS);
            sleep(1500);
            servoPlacer.setPosition(0.65 * MAX_POS);
            sleep(500);

            //lower the lift while placing
            liftspeed = 0.25;
            liftRight.setPower(-liftspeed);
            liftLeft.setPower(-liftspeed);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }


        } else if (DESIRED_TAG_ID == 4) {
            runMotorsWithTurning(0, 0, -90); //rotate clockwise 90 degree
            //sleep(1000);
            //drop pixel here
            IntakeWheel.setPower(-0.25);
            sleep(1000);
            IntakeWheel.setPower(0);
            //drive toward backdrop to Apirl Tag ID 4
            runMotorsForTime(0, -0.3, 0, 1.7);

//            DrivetoApriltag(DESIRED_TAG_ID);
//
            encoderDrive(0.65, 34, 34, 34, 34, 4);//                //runMotorsWithTurning(0, 0, -90); //rotate clockwise 90 degree
            //drop the pixel on backstage score 3 points
            double liftspeed = 0.3;
            //lift goes up with runtime of 2 second
            liftRight.setPower(liftspeed);
            liftLeft.setPower(liftspeed);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            //place the pixel
            servoPlacer.setPosition(MIN_POS);
            sleep(1500);
            servoPlacer.setPosition(0.65 * MAX_POS);
            sleep(500);

            //lower the lift while placing
            liftspeed = 0.25;
            liftRight.setPower(-liftspeed);
            liftLeft.setPower(-liftspeed);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        }
//        telemetry.addData("autonomous DONE for DESIRED_TAG_ID = ", DESIRED_TAG_ID);
//        telemetry.update();
//        sleep(1000);
    }

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }


    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double fleftInches, double frightInches,
                             double rleftInches, double rrightInches,
                             double timeoutS) {
        int newfLeftTarget;
        int newrLeftTarget;
        int newfRightTarget;
        int newrRightTarget;
//        telemetry.addData(">","in encoderDrive");
//        telemetry.update();
//        sleep(1000);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
//            telemetry.addData(">","encoderDrive Active");
//            telemetry.update();
//            sleep(1000);

            // Determine new target position, and pass to motor controller
            newfLeftTarget = frontLeft.getCurrentPosition() + (int) (fleftInches * COUNTS_PER_INCH);
            newfRightTarget = frontRight.getCurrentPosition() + (int) (frightInches * COUNTS_PER_INCH);
            newrLeftTarget = rearLeft.getCurrentPosition() + (int) (rleftInches * COUNTS_PER_INCH);
            newrRightTarget = rearRight.getCurrentPosition() + (int) (rrightInches * COUNTS_PER_INCH);


            frontLeft.setTargetPosition(newfLeftTarget);
            frontRight.setTargetPosition(newfRightTarget);
            rearLeft.setTargetPosition(newrLeftTarget);
            rearRight.setTargetPosition(newrRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            rearLeft.setPower(Math.abs(speed));
            rearRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newfLeftTarget, newfRightTarget, newrLeftTarget, newrRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), rearLeft.getCurrentPosition(), rearRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderLeft(double speed,
                            double fleftInches, double frightInches,
                            double rleftInches, double rrightInches,
                            double timeoutS) {
        int newfLeftTarget;
        int newrLeftTarget;
        int newfRightTarget;
        int newrRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfLeftTarget = frontLeft.getCurrentPosition(); //+ (int)(fleftInches * COUNTS_PER_INCH);
            newfRightTarget = frontRight.getCurrentPosition();// + (int)(frightInches * COUNTS_PER_INCH);
            newrLeftTarget = rearLeft.getCurrentPosition();// + (int)(rleftInches * COUNTS_PER_INCH);
            newrRightTarget = rearRight.getCurrentPosition();// + (int)(rrightInches * COUNTS_PER_INCH);

            //this following code moved strafing to the right 11 inches
            frontLeft.setTargetPosition(newfLeftTarget - 500);
            frontRight.setTargetPosition(newfRightTarget + 500);
            rearLeft.setTargetPosition(newrLeftTarget + 500);
            rearRight.setTargetPosition(newrRightTarget - 500);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            rearLeft.setPower(Math.abs(speed));
            rearRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newfLeftTarget, newfRightTarget, newrLeftTarget, newrRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), rearLeft.getCurrentPosition(), rearRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        rearLeft.setPower(leftBackPower);
        rearRight.setPower(rightBackPower);
    }
}
