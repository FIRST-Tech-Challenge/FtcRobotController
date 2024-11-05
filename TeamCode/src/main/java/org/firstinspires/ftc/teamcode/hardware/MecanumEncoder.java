package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumEncoder {
    private class BotSettings {
        BotSettings(double countsPerMoterRev, double driveGearReduction, double wheelDiameterInches, RevHubOrientationOnRobot revHubOrientation){
            this.COUNTS_PER_MOTOR_REV = countsPerMoterRev;
            this.DRIVE_GEAR_REDUCTION = driveGearReduction;
            this.WHEEL_DIAMETER_INCHES = wheelDiameterInches;
            this.REV_HUB_ORIENTATION = revHubOrientation;
        }
        double COUNTS_PER_MOTOR_REV    = 0.0 ;    // eg: TETRIX Motor Encoder
        double DRIVE_GEAR_REDUCTION    = 0.0 ;     // No External Gearing.
        double WHEEL_DIAMETER_INCHES   = 0.0 ;     // For figuring circumference
        RevHubOrientationOnRobot REV_HUB_ORIENTATION = null;
        double CountsPerInch() {
            return (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        }
        RevHubOrientationOnRobot GetRevHubOrientation() {
            return REV_HUB_ORIENTATION;
        }
    }

    BotSettings testBot = new BotSettings(537.7, 1.0, 3.178, new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    BotSettings compBot = new BotSettings(537.7, 1.0, 4.095, new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));
    BotSettings settings = null;

    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    //private DcMotorEx LiftMotor;

    private Telemetry.Item lift_status_tel = null;
    private Telemetry.Item liftamount_tel = null;
    boolean lift_status = false;
    double liftamount = 0.0;
    Gamepad previous = new Gamepad();
    Gamepad current = new Gamepad();


    private ElapsedTime     runtime = new ElapsedTime();
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.8;

    private OpMode opMode;
    private  IMU imu;

    public enum DriveMode {FieldCentric, BotCentric}
    public enum Bot {TestBot, CompBot}

    public MecanumEncoder(OpMode opMode)
    {
        this.opMode = opMode;
    }

    public IMU getImu() {
        return imu;
    }

    public void setImu(IMU imu) {
        this.imu = imu;
    }

    public void initHardware(HardwareMap hardwareMap, Bot bot)
    {
        //Read controllerhubid.txt at root of hub file system
        //Filepath for Windows = Control Hub v1.0\Internal shared storage\controllerhubid.txt
        //Read value in file test = test bot profile
        //Read value in file comp = comp bot profile
                        //
        settings = (bot == Bot.TestBot) ? testBot : compBot;
        frontLeft  = hardwareMap.get(DcMotorEx.class, "flmotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frmotor");
        backLeft  = hardwareMap.get(DcMotorEx.class, "blmotor");
        backRight = hardwareMap.get(DcMotorEx.class, "brmotor");
        //LiftMotor = hardwareMap.get(DcMotorEx.class, "liftmotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE); // REVERSE
        frontRight.setDirection(DcMotor.Direction.FORWARD); // FORWARD
        backLeft.setDirection(DcMotor.Direction.REVERSE); // REVERSE
        backRight.setDirection(DcMotor.Direction.FORWARD); // FORWARD
        //LiftMotor.setDirection(DcMotor.Direction.FORWARD); // FORWARD

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        IMU.Parameters parameters = new IMU.Parameters(settings.GetRevHubOrientation());
        imu.initialize(parameters);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void stop() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        //LiftMotor.setPower(0);

    }

    public void driverInput(double leftX, double leftY, double rightX, double maxPower, DriveMode driveMode) {
        double y = -leftY; // Remember, Y stick value is reversed
        double x = leftX;
        double rx = rightX;


        double botHeading = 0.0;
        if(driveMode == DriveMode.FieldCentric)
        {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        double rotX = 0.0;
        double rotY = 0.0;

        // Rotate the movement direction counter to the bot's rotation
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.0;  // Counteract imperfect strafing

        double frontLeftPower = (rotY + rotX + rx);
        double backLeftPower = (rotY - rotX + rx);
        double frontRightPower = (rotY - rotX - rx);
        double backRightPower = (rotY + rotX - rx);

        //scale the powers
        double scaleFactor = maxPower; //default scale value
        double maxCalculatedPower =
                Math.max(
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                );

        if(maxCalculatedPower > 1.0) {
            //max calculated power becomes the new scaler
            scaleFactor = maxCalculatedPower;
        }

        //scall all the power values
        frontLeftPower = frontLeftPower/scaleFactor;
        frontRightPower = frontRightPower/scaleFactor;
        backLeftPower = backLeftPower/scaleFactor;
        backRightPower = backRightPower/scaleFactor;

        drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    public void drive(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        backRight.getVelocity()
    }

    public void drive(double frontLeftPower, double frontRightPower,
                      double backLeftPower, double backRightPower,
                      int frontLeftPos, int frontRightPos, int backLeftPos, int backRightPos,
                      double timeoutSec) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(frontLeftPower));
        frontRight.setPower(Math.abs(frontRightPower));
        backLeft.setPower(Math.abs(backLeftPower));
        backRight.setPower(Math.abs(backRightPower));

        runtime.reset();
        while(
                (
                        frontLeft.isBusy() || frontRight.isBusy() ||
                                backLeft.isBusy() || backRight.isBusy()
                )
                        &&
                        runtime.seconds() < timeoutSec){

//            opMode.telemetry.addData("Tar",  "fl:%7d fr:%7d  bl:%7d  br:%7d",
//                frontLeft.getTargetPosition(), frontRight.getTargetPosition(),
//                backLeft.getTargetPosition(), backRight.getTargetPosition());
//            opMode.telemetry.addData("Cur",  "fl:%7d fr:%7d  bl:%7d  br:%7d",
//                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
//                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
//            opMode.telemetry.update();

        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Turn off RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveForwardInches(double power, double inches, double timeoutSec){

        int fl = (int)(inches * settings.CountsPerInch());
        int fr = (int)(inches * settings.CountsPerInch());
        int bl = (int)(inches * settings.CountsPerInch());
        int br = (int)(inches * settings.CountsPerInch());

        drive(power, power, power, power, fl, fr, bl, br, timeoutSec);
    }

    public void driveBackwardInches(double power,  double inches,
                                    double timeoutSec){

        int fl = (int)(0 - (inches * settings.CountsPerInch()));
        int fr = (int)(0 - (inches * settings.CountsPerInch()));
        int bl = (int)(0 - (inches * settings.CountsPerInch()));
        int br = (int)(0 - (inches * settings.CountsPerInch()));

        drive(power, power, power, power, fl, fr, bl, br, timeoutSec);
    }
    public void driveLeftInches(double power,  double inches,
                                double timeoutSec){

        int fl = (int)(0 - (inches * settings.CountsPerInch()));
        int fr = (int)(inches * settings.CountsPerInch());
        int bl = (int)(inches * settings.CountsPerInch());
        int br = (int)(0 - (inches * settings.CountsPerInch()));

        drive(power, power, power, power, fl, fr, bl, br, timeoutSec);
    }
    public void driveRightInches(double power,  double inches,
                                 double timeoutSec){

        int fl = (int)(inches * settings.CountsPerInch());
        int fr = (int)(0 - (inches * settings.CountsPerInch()));
        int bl = (int)(0 - (inches * settings.CountsPerInch()));
        int br = (int)(inches * settings.CountsPerInch());

        drive(power, power, power, power, fl, fr, bl, br, timeoutSec);
    }

    public void rotateClockwise(double power, double inches,
                                double timeoutSec){

        int fl = (int)(inches * settings.CountsPerInch());
        int fr = (int)(0 - (inches * settings.CountsPerInch()));
        int bl = (int)(inches * settings.CountsPerInch());
        int br = (int)(0 - (inches * settings.CountsPerInch()));

        drive(power, power, power, power, fl, fr, bl, br, timeoutSec);
    }

//    public void rotateClockwiseDegrees(double power, double degrees, double timeoutSec){
//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        double targetHeading = botHeading + degrees;
//        double powerFactor = 1.0;
//        while( Math.abs(targetHeading - botHeading) < 1) {
//            double degreesToMove = Math.abs(targetHeading - botHeading);
//            if(degreesToMove < 5) powerFactor = .5;
//            if(targetHeading - botHeading > 1){
//                drive(power*powerFactor, power*powerFactor, power*powerFactor, power*powerFactor);
//                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            }
//            if(targetHeading - botHeading < 1){
//                drive(-power*powerFactor, -power*powerFactor, -power*powerFactor, -power*powerFactor);
//                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            }
//        }
//    }

    public void rotateCounterClockwise(double power, double inches,
                                       double timeoutSec){

        int fl = (int)(0 - (inches * settings.CountsPerInch()));
        int fr = (int)(inches * settings.CountsPerInch());
        int bl = (int)(0 - (inches * settings.CountsPerInch()));
        int br = (int)(inches * settings.CountsPerInch());
//        while (opModeInInit()) {
//            current.copy(gamepad1);
//            if (current.b && !previous.b) {
//                lift_status = !lift_status;
//            }

        drive(power, power, power, power, fl, fr, bl, br, timeoutSec);
    }

    //Lift Servo
//    public void liftmotor() {
//        previous.copy(gamepad1);
//        current.copy(gamepad1);
//
//            if (lift_status == true) {
//
//                if (current.dpad_up && !previous.dpad_up) {
//                    LiftMotor.setPower(0);
//            }
//
//
//                if (current.dpad_down && !previous.dpad_down) {
//                    LiftMotor.setPower(100);
//            }
//
//            parking_status_tel.setValue(parking_status ? "true" : "false");
//            parking_delay_tel.setValue(parking_delay);
//            telemetry.update();
//            previous.copy(current);
//        }
//    }

}
