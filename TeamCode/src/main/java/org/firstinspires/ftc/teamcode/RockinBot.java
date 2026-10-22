package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;

// All the things that we use and borrow
import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//These are for FTC Dashboard. However, we may be missing an import or two
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
// import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RockinBot {
    // Motors and sensors
    private LinearOpMode o;
    private DcMotorEx intake = null;
    private Pose2D pos;
    // private Pose3D botPose;
    private double xLoc = 0;
    private double yLoc = 0;
    private double hLoc = 0;
    private DcMotorEx shooter = null;
    private PIDFCoefficients pidf = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor trigger = null;
    private Servo lumberjack = null;
    // private Limelight3A limelight;
    public GoBildaPinpointDriver odo = null;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double intakeCurrent = 0;
    private double shooterPower = 0;
    private double max = 0;
    private double triggerPower = 0;
    double shooterSpeed = 1.0;
    double intakePower = 1.0;
    // These do NOT affect anything, but leave them as is! See notes in RemoteControlShooter for more information
    // These should be affecting RC, but they do not, and we fear that if we change them, everything will explode
    final ElapsedTime runtime = new ElapsedTime();
    final ElapsedTime pinpointTime = new ElapsedTime();

    // During runtime

    public RockinBot(LinearOpMode opMode) {
        o = opMode;
        o.telemetry.addData("Version", "v20261013-03");
        o.telemetry.update();
        initializeVars();
    }

    // Allow driving and braking
    public void initializeVars() {
        // Wheel variables
        leftFrontDrive = o.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = o.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = o.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = o.hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initializes the pinpoint
        odo = o.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        final double X_POD_OFFSET_MM = 114.3;
        final double Y_POD_OFFSET_MM = -165.1;
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.resetPosAndIMU();
        odo.update();

        intake = o.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooter = o.hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidf = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        setpValue(55); // this often gets overwritten by other programs

        trigger = o.hardwareMap.get(DcMotorEx.class, "trigger");
        trigger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trigger.setDirection(DcMotor.Direction.REVERSE);

        lumberjack = o.hardwareMap.get(Servo.class, "lumberjack");

        // limelight = o.hardwareMap.get(Limelight3A.class, "limelight");
        // limelight.pipelineSwitch(0);
        // RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
        //         RevHubOrientationOnRobot.LogoFacingDirection.UP,
        //         RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        RobotLog.vv("Rockin' Robots", "Hardware Initialized");
    }

    // public void startLimelight()
    // {
    //     limelight.start();
    // }

    // public void loopLimelight(){
    //     odo.update();
    //     limelight.updateRobotOrientation(odo.getHeading());
    //     LLResult result = limelight.getLatestResult();
    //     if(result != null && result.isValid())
    //     {
    //         o.telemetry.addData("X", result.getTx());
    //         o.telemetry.addData("Y", result.getTy());
    //         o.telemetry.addData("A", result.getTa());
    //         o.telemetry.addData("BotPose", botPose);
    //         o.telemetry.addData("Yaw", botPose.getOrientation().getYaw());
    //         getPinpointPosition();
    //     }
    // }
    // Remote control driving functions
    public void setWheelPower(double leftStickY, double leftStickX, double rightStickX) {
        double wheelMultiplier = 0.85;
        double forward = leftStickY;
        double strafe = -leftStickX;

        leftFrontPower = (forward + strafe + rightStickX) * wheelMultiplier;
        rightFrontPower = (forward - strafe - rightStickX) * wheelMultiplier;
        leftBackPower = (forward - strafe + rightStickX) * wheelMultiplier;
        rightBackPower = (forward + strafe - rightStickX) * wheelMultiplier;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        } else if (max <= 0.05) {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        RobotLog.vv("Rockin' Robots", "Wheel power: %.2f, %.2f, %.2f, %.2f",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void adjustpValue(double delta) {
        pidf.p = Math.max(0.0, pidf.p + delta);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        RobotLog.vv("Rockin' Robots", "PIDF changed. New p value: " + pidf.p);
    }

    public void setpValue(double pValue) {
        pidf.p = Math.max(0.0, pValue);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        RobotLog.vv("Rockin' Robots", "PIDF changed. New p value: " + pidf.p);
    }

    public void setLumberjackPosition(double position) {
        lumberjack.setPosition(position);
        RobotLog.vv("Rockin' Robots", "Lumberjack position: %.2f", position);
    }

    public void releaseLumberjack() {
        ServoController controller = lumberjack.getController();
        if (!(controller instanceof ServoControllerEx)) {
            RobotLog.ee("Rockin' Robots", "Cannot release only the lumberjack: servo controller does not support per-port PWM");
            return;
        }

        ((ServoControllerEx) controller).setServoPwmDisable(lumberjack.getPortNumber());
        RobotLog.vv("Rockin' Robots", "Lumberjack PWM disabled");
    }

    public void shooterVelocity(double vel) {
        RobotLog.vv("Rockin' Robots", "shooterPower(%.2f)", vel);
        shooterSpeed = vel;
        shooter.setVelocity(vel);
    }

    public boolean getPinpointPosition() {     // Finds robot position
        pinpointTime.reset();

        // Wait until Pinpoint is ready (up to 1s total).
        while (odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.NOT_READY
                && pinpointTime.seconds() < 1) {
            RobotLog.vv("Rockin' Robots", "Pinpoint NOT_READY, status=" + odo.getDeviceStatus());
            stopMoving();
            sleep(10);
        }
        if (odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
            RobotLog.vv("Rockin' Robots", "getPinpointPosition: timed out waiting for READY");
            return false;
        }

        // Read until we get non-NaN values, sharing the same 1s budget.
        do {
            odo.update();
            pos = odo.getPosition();
            xLoc = pos.getX(DistanceUnit.MM);
            yLoc = pos.getY(DistanceUnit.MM);
            hLoc = pos.getHeading(AngleUnit.DEGREES);
            if (!Double.isNaN(xLoc) && !Double.isNaN(yLoc) && !Double.isNaN(hLoc)) {
                RobotLog.vv("Rockin' Robots",
                        "New Position: {X: %.1f, Y: %.1f, H: %.1f}", xLoc, yLoc, hLoc);
                return true;
            }
            RobotLog.vv("Rockin' Robots", "Pinpoint returned NaN, status=" + odo.getDeviceStatus());
            stopMoving();
            sleep(10);
        } while (pinpointTime.seconds() < 1);

        RobotLog.vv("Rockin' Robots", "getPinpointPosition: timed out with NaN values");
        return false;
    }

    public void stopMoving() {
        RobotLog.vv("Rockin' Robots", "stopMoving()");
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void driveForward(int ms) {
        leftFrontDrive.setPower(-0.5); // clock
        rightFrontDrive.setPower(-0.5); // counter
        leftBackDrive.setPower(-0.5); // counter
        rightBackDrive.setPower(-0.5); // clock
        sleep(ms);
        stopMoving();
    }

    public void driveBack(int ms) {
        leftFrontDrive.setPower(0.5); // counter
        rightFrontDrive.setPower(0.5); // clock
        leftBackDrive.setPower(0.5); // clock
        rightBackDrive.setPower(0.5); // counter
        sleep(ms);
        stopMoving();
    }

    public void driveRight(int ms) {
        leftFrontDrive.setPower(0.5); // clock
        rightFrontDrive.setPower(0.5); // clock
        leftBackDrive.setPower(-0.5); // clock
        rightBackDrive.setPower(-0.5); // clock
        sleep(ms);
        stopMoving();
    }

    public void driveToPos(double xTarget, double yTarget, double hTarget,
                           double xyAccuracy, double hAccuracy, double maxDuration) {
        driveToPos(xTarget, yTarget, hTarget, xyAccuracy, hAccuracy, maxDuration, 1.0);
    }

    public void driveToPos(double xTarget, double yTarget, double hTarget,
                           double xyAccuracy, double hAccuracy, double maxDuration,
                           double maxDriveSpeed) {
        if (!getPinpointPosition()) {
            RobotLog.vv("Rockin' Robots", "driveToPos ABORT: initial Pinpoint read failed");
            return;
        }

        double xDistance = xTarget - xLoc;
        double yDistance = yTarget - yLoc;
        double hDistance = hTarget - hLoc;
        if (hDistance > 180) hDistance -= 360;
        if (hDistance < -180) hDistance += 360;

        runtime.reset();
        while (o.opModeIsActive()
                && runtime.seconds() < maxDuration
                && (Math.abs(xDistance) > xyAccuracy
                || Math.abs(yDistance) > xyAccuracy
                || Math.abs(hDistance) > hAccuracy)) {
            double angleRadians = Math.toRadians(hLoc);
            double xRotatedDistance = xDistance * Math.cos(angleRadians)
                    + yDistance * Math.sin(angleRadians);
            double yRotatedDistance = -xDistance * Math.sin(angleRadians)
                    + yDistance * Math.cos(angleRadians);

            double xCmd = xRotatedDistance * 0.003;
            double yCmd = yRotatedDistance * 0.003;
            double hCmd = hDistance * 0.03;

            leftFrontPower = yCmd + xCmd - hCmd;
            rightFrontPower = yCmd - xCmd + hCmd;
            leftBackPower = yCmd - xCmd - hCmd;
            rightBackPower = yCmd + xCmd + hCmd;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > maxDriveSpeed) {
                double scale = maxDriveSpeed / max;
                leftFrontPower *= scale;
                rightFrontPower *= scale;
                leftBackPower *= scale;
                rightBackPower *= scale;
            } else if (max > 0.001 && max < 0.15) {
                double boost = 0.15 / max;
                leftFrontPower *= boost;
                rightFrontPower *= boost;
                leftBackPower *= boost;
                rightBackPower *= boost;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if (!getPinpointPosition()) {
                RobotLog.vv("Rockin' Robots", "driveToPos ABORT: Pinpoint read failed mid-move");
                stopMoving();
                return;
            }

            xDistance = xTarget - xLoc;
            yDistance = yTarget - yLoc;
            hDistance = hTarget - hLoc;
            if (hDistance > 180) hDistance -= 360;
            if (hDistance < -180) hDistance += 360;
            o.idle();
        }
        stopMoving();
    }

    public void intakePower(double power) {
        RobotLog.vv("Rockin' Robots", "intakePower(%.2f)", power);
        intakePower = power;
        intake.setPower(power);
    }

    public void triggerPower(double power) {
        triggerPower = power;
        trigger.setPower(power);
    }

    private boolean inBand(double actual, double lower, double upper) {
        return actual >= lower && actual <= upper;
    }

    public void printDataOnScreen() {

        intakeCurrent = intake.getCurrent(CurrentUnit.MILLIAMPS);

        shooterPower = shooter.getCurrent(CurrentUnit.MILLIAMPS);
        o.telemetry.addData("Shooter speed and Power", "%.2f, %.0f mA", shooterSpeed, shooterPower);
        o.telemetry.addData("Intake power and current", "%.2f, %.0f mA", intakePower, intakeCurrent);
        o.telemetry.addData("Trigger Power", "%.2f", triggerPower);
        o.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        o.telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        o.telemetry.addData("P value", "%.2f", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
        o.telemetry.addData("Lumberjack position", "%.2f", lumberjack.getPosition());
        // loop();

        o.telemetry.update();

        /*
        These are also for FTC Dashboard
        dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("Left Launcher:", leftLauncherVelocity);
        dashboardTelemetry.addData("Right Launcher:", rightLauncherVelocity);
        dashboardTelemetry.addData("Intake current", intakeCurrent);
        dashboardTelemetry.addData("Left Front Wheel", leftFrontPower);
        dashboardTelemetry.addData("Right Front Wheel", rightFrontPower);
        dashboardTelemetry.addData("Left Back Wheel", leftBackPower);
        dashboardTelemetry.addData("Right Back Wheel", rightBackPower);
        dashboardTelemetry.update();*/
    }
}
