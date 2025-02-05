package IntoTheDeep2024;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="24 Auto Drive", group="Robot")
@Disabled
public class OTOSAutoDrive extends LinearOpMode {
    // Initialize all variables for the program
    // Hardware variables
    SparkFunOTOS myOtos;
    double xLoc = 0;
    double yLoc = 0;
    double hLoc = 0;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    double max = 0;

    // This chunk controls our vertical
    DcMotor vertical = null;
    final int VERTICAL_MIN = 0;
    final int VERTICAL_MAX = 1700;
    final int VERTICAL_DEFAULT_SPEED = 3000;

    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MIN = 0;
    final int VIPER_MAX = 3100;
    final int VIPER_GROUND = 1000;
    final int VIPER_DEFAULT_SPEED = 3000;

    // This chunk controls our claw
    //Callie
    Servo claw = null;
    final double CLAW_MIN = 0.4;           // Claw is closed
    final double CLAW_MAX = 0.93;           // Claw is open

    Servo ascentStick = null;
    final double ASCENT_MIN = 0.17;          // Stick is down
    final double ASCENT_MAX = 0.43;         // Stick is up

    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeHardwareVariables();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous Ready", "You can press start");
        telemetry.addData("This code was last updated", "01/04/2025, 5:39 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        configureOtos();

        // First Sample ///////////////////////////////////////////////////////////////
        setVertical(VERTICAL_MAX);                                  // Raising Arm
        setViper(VIPER_MAX);                                        // Extending Viper
        sleep(500);
        driveToLoc(9, 14, 45, 1);  // Go to basket
        sleep(100);
        setClaw(CLAW_MAX);                                          // Drop the block
        sleep(300);

        // Second Sample ///////////////////////////////////////////////////////////////
        RobotLog.vv("Rockin'", "Get Sample 2");
        driveToLoc(15, 12, -10, 4);
        setViper(VIPER_GROUND);
        sleep(300);
        setVertical(VERTICAL_MIN);
        driveToLoc(23, 0, -25, 1);
        driveToLoc(24, 4, -25, 0.5);  // Move forward to get block
        sleep(100);
        setClaw(CLAW_MIN);                                          // Grab second block
        sleep(200);
        setVertical(VERTICAL_MAX);
        setViper(VIPER_MAX);
        driveToLoc(9, 12, 45, 1);  // Go to basket
        sleep(300);
        setClaw(CLAW_MAX);                                          // Drop second block
        sleep(200);

        // Third Sample ///////////////////////////////////////////////////////////////
        RobotLog.vv("Rockin'", "Get Sample 3");
        driveToLoc(16, 10, -10, 4);
        setViper(VIPER_GROUND);
        sleep(300);
        setVertical(VERTICAL_MIN);
        driveToLoc(22, 4, -20, 0.5);
        driveToLoc(24, 11, -20, 0.5);  // Move forward to get block
        sleep(100);
        setClaw(CLAW_MIN);                                          // Grab third block
        sleep(200);
        setVertical(VERTICAL_MAX);
        setViper(VIPER_MAX);
        driveToLoc(8, 13, 45, 1);  // Go to basket
        sleep(200);
        setClaw(CLAW_MAX);                                          // Drop third block
        sleep(300);

        // Fourth Sample ///////////////////////////////////////////////////////////////
        driveToLoc(23, 13, -20, 2);
        setViper(VIPER_GROUND);
        sleep(1000);
        setVertical(VERTICAL_MIN);
        sleep(1000);
        driveToLoc(23, 19, -20, 0.5);  // Move forward to get block
        sleep(100);
        setClaw(CLAW_MIN);                                          // Grab fourth block
        sleep(100);
        driveToLoc(24, 12, -20);
        setVertical(VERTICAL_MAX);
        setViper(VIPER_MAX);
        driveToLoc(8, 14, 45, 1);  // Go to basket
        sleep(200);
        setClaw(CLAW_MAX);                                          // Drop fourth block

        // Park ///////////////////////////////////////////////////////////////
        driveToLoc(25, 5, 0, 3);
        setViper(VIPER_MIN);
        sleep(700);
        setVertical(VERTICAL_MIN);
        driveToLoc(50, 5, 0);
        setAscentStick(ASCENT_MAX);
        driveToLoc(50, -10, 180);
        sleep(300);
        RobotLog.vv("Rockin'", "End program");
        claw.close();                                               // Release tension on the claw
        // End of autonomous program
        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }

    public void setAscentStick(double target) {
        RobotLog.vv("Rockin' Robots", "setAscentStick() target: %4.2f, current: %4.2f", target, ascentStick.getPosition());
        ascentStick.setPosition(target);
    }

    public void setViper(int length){ setViper(length, VIPER_DEFAULT_SPEED); }

    public void setViper(int length, int velocity){
        RobotLog.vv("Rockin' Robots", "setViper() length: %d, current: %d", length, viperSlide.getCurrentPosition());
        viperSlide.setTargetPosition(length);
        ((DcMotorEx) viperSlide).setVelocity(velocity);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setVertical(int height){
        setVertical(height, VERTICAL_DEFAULT_SPEED);
    }

    public void setVertical(int height, int speed){
        RobotLog.vv("Rockin' Robots", "setVertical() height: %d, speed: %d, current: %d", height, speed, vertical.getCurrentPosition());
        vertical.setTargetPosition(height);
        ((DcMotorEx) vertical).setVelocity(speed);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (height == VERTICAL_MAX) { sleep(500); }
    }

    public void setClaw(double position) {
        RobotLog.vv("Rockin' Robots", "setClaw() position: %4.2f, current: %4.2f", position, claw.getPosition());
        claw.setPosition(position);
        sleep(400);
    }

    private void moveForward(double speed, long msDuration) {
        RobotLog.vv("Rockin' Robots", "moveForward() speed: %4.2f, msDuration:: %d", speed, msDuration);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        sleep(msDuration);
        stopMoving();
    }

    private void turnRight(double speed, double target) {
        RobotLog.vv("Rockin' Robots", "turnRight() speed: %4.2f, target: %4.2f, current: %4.2f", speed, target, hLoc);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(-speed);
        getPosition();
        while(hLoc < target -2 || hLoc > target + 2) {
            sleep(10);
            getPosition();
            if(hLoc > 180) hLoc -= 360;
            if(hLoc < -180) hLoc += 360;
        }
        stopMoving();
    }

    private void stopMoving() {
        RobotLog.vv("Rockin' Robots", "stopMoving()");
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.5, 1.1, 90);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);
    }

    private void getPosition() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        xLoc = pos.x;
        yLoc = pos.y;
        hLoc = pos.h;
    }

    private void driveToLoc(double xTarget, double yTarget, double hTarget) {
        driveToLoc(xTarget, yTarget, hTarget, 2);
    }

    private void driveToLoc(double xTarget, double yTarget, double hTarget, double accuracy) {
        getPosition();
        double xDistance = xTarget - xLoc;
        double yDistance = yTarget - yLoc;
        double hDistance = hTarget - hLoc;
        if(hDistance > 180) hDistance -= 360;
        if(hDistance < -180) hDistance += 360;
        double angleRadians = Math.toRadians(hLoc);
        double xRotatedDistance = xDistance * Math.cos(angleRadians) + yDistance * Math.sin(angleRadians);
        double yRotatedDistance = - xDistance * Math.sin(angleRadians) + yDistance * Math.cos(angleRadians);

        RobotLog.vv("Rockin' Robots", "driveToLoc() xTarget: %.2f, yTarget: %.2f, hTarget: %.2f, accuracy: %.2f",
                xTarget, yTarget, hTarget, accuracy);

        runtime.reset();
        while (opModeIsActive()
                &&(runtime.seconds() < 5)
                && (Math.abs(xDistance) > accuracy
                || Math.abs(yDistance) > accuracy
                || Math.abs(hDistance) > accuracy)) {

            leftFrontPower = (yRotatedDistance + xRotatedDistance - hDistance) / 8;
            rightFrontPower = (yRotatedDistance - xRotatedDistance + hDistance) / 8;
            leftBackPower = (yRotatedDistance - xRotatedDistance - hDistance) / 8;
            rightBackPower = (yRotatedDistance + xRotatedDistance + hDistance) / 8;

            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            leftFrontPower *= 0.7;
            rightFrontPower *= 0.7;
            leftBackPower *= 0.7;
            rightBackPower *= 0.7;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max < 0.15) {
                leftFrontPower *= 1.5;
                rightFrontPower *= 1.5;
                leftBackPower *= 1.5;
                rightBackPower *= 1.5;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            RobotLog.vv("Rockin' Robots", "xDist: %.2f, yDist: %.2f, hDist: %.2f, " +
                    "leftFrontPower: %.2f, rightFrontPower: %.2f, leftBackPower: %.2f, rightBackPower: %.2f",
                    xDistance, yDistance, hDistance, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            getPosition();
            xDistance = xTarget - xLoc;
            yDistance = yTarget - yLoc;
            hDistance = hTarget - hLoc;
            if(hDistance > 180) hDistance -= 360;
            if(hDistance < -180) hDistance += 360;

            angleRadians = Math.toRadians(hLoc);
            xRotatedDistance = xDistance * Math.cos(angleRadians) + yDistance * Math.sin(angleRadians);
            yRotatedDistance = - xDistance * Math.sin(angleRadians) + yDistance * Math.cos(angleRadians);
        }
        stopMoving();
        RobotLog.vv("Rockin' Robots", "Done Moving: xDist: %.2f, yDist: %.2f, hDist: %.2f",
                xDistance, yDistance, hDistance);
    }

    public void initializeHardwareVariables() {
        // Define all the hardware
        myOtos = hardwareMap.get(SparkFunOTOS.class, "OTOS");
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setTargetPosition(0);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperSlide = hardwareMap.get(DcMotor.class, "viper_slide");
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlide.setDirection(DcMotor.Direction.REVERSE);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setTargetPosition(0);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(CLAW_MIN);

        ascentStick = hardwareMap.get(Servo.class, "ascentStick");
        ascentStick.setDirection(Servo.Direction.REVERSE);
        ascentStick.setPosition(ASCENT_MIN);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    }

}