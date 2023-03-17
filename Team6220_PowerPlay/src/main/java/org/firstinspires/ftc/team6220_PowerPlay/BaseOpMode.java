package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

public abstract class BaseOpMode extends LinearOpMode {
    // motors
    public static DcMotorEx motorFL;
    public static DcMotorEx motorFR;
    public static DcMotorEx motorBL;
    public static DcMotorEx motorBR;

    public static DcMotorEx motorLeftSlides;
    public static DcMotorEx motorRightSlides;

    // servos
    public static ServoImplEx servoGrabber;
    public static RevBlinkinLedDriver blinkinChassis;

    // OpenCV
    public static OpenCvCamera robotCamera;
    public static OpenCvCamera grabberCamera;
    public static AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public static RobotCameraPipeline robotCameraPipeline;
    public static GrabberCameraPipeline grabberCameraPipeline;

    // IMU
    public BNO055IMU imu;
    public double originalAngle;
    public double startAngle;

    // flag to say whether we should disable the correction system
    private boolean turnFlag = false;

    // bulk reading
    private List<LynxModule> hubs;

    // initializes the motors, servos, and IMUs
    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // motors
        motorFL = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorBR");
        motorLeftSlides = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorLeftSlides");
        motorRightSlides = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorRightSlides");

        motorFL.setDirection(DcMotorEx.Direction.FORWARD);
        motorFR.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.FORWARD);
        motorBR.setDirection(DcMotorEx.Direction.REVERSE);

        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorLeftSlides.setDirection(DcMotorEx.Direction.FORWARD);
        motorRightSlides.setDirection(DcMotorEx.Direction.REVERSE);

        motorLeftSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motorLeftSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRightSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // servos
        PwmControl.PwmRange maxRange = new PwmControl.PwmRange(505, 2495, 20000);
        servoGrabber = (ServoImplEx) hardwareMap.servo.get("servoGrabber");
        servoGrabber.setPwmRange(maxRange);

        try {
            blinkinChassis = (RevBlinkinLedDriver) hardwareMap.get(RevBlinkinLedDriver.class, "blinkinChassis");
        } catch (Exception e) {}

        // initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // sleep to allow the IMU to initialize before the absolute angles are measured
        sleep(3000);

        startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        originalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"));
        grabberCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "GrabberCamera"), cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();
        robotCameraPipeline = new RobotCameraPipeline();
        grabberCameraPipeline = new GrabberCameraPipeline();

        servoGrabber.setPosition(Constants.GRABBER_INITIALIZE_POSITION);
    }

    /**
     * drives the robot with the IMU
     * when not turning, the robot maintains a constant heading
     * @param xPower the motor power for moving in the x-direction
     * @param yPower the motor power for moving in the y-direction
     * @param tPower the motor power for pivoting
     */
    public void driveWithIMU(double xPower, double yPower, double tPower) {
        // read imu when turning (when t != 0)
        boolean isTurning = tPower != 0;

        if (isTurning || turnFlag) {
            originalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // set original angle
            turnFlag = true;

        // otherwise read imu for correction
        } else {
            // obtain the current angle's error from the original angle
            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double angleError = originalAngle - currentAngle;

            // flip to inverse of angles above 180 / below -180 (to prevent infinity-rotate bug)
            // to make sure to use the shorter angle
            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }

            // apply a constant to turn the angle into a turn speed
            tPower = Math.min(Math.abs(-Constants.HEADING_CORRECTION_KP_TELEOP * angleError), Constants.MAXIMUM_TURN_POWER_TELEOP) * Math.signum(-angleError);
        }

        // if the rotation rate is low, then that means all the momentum has left the robot's turning and can therefore turn the correction back on
        if (Math.abs(imu.getAngularVelocity().zRotationRate) < 5) {
            turnFlag = false;
        }

        // calculate speed and direction of each individual motor and set power of motors to speed
        motorFL.setPower(yPower + xPower + tPower);
        motorFR.setPower(yPower - xPower - tPower);
        motorBL.setPower(yPower - xPower + tPower);
        motorBR.setPower(yPower + xPower - tPower);
    }

    public void driveWithoutIMU(double xPower, double yPower, double tPower) {
        motorFL.setPower(yPower + xPower + tPower);
        motorFR.setPower(yPower - xPower - tPower);
        motorBL.setPower(yPower - xPower + tPower);
        motorBR.setPower(yPower + xPower - tPower);
    }

    /**
     * this method will allow the grabber to open or close given a boolean input
     * @param position the position of the grabber
     */
    public void driveGrabber(double position) {
        servoGrabber.setPosition(position);
    }

    /**
     * this method will allow the slides to move to a specified target position
     * @param targetPosition target position for slides motors in ticks
     */
    public void driveSlides(int targetPosition) {
        int error = targetPosition - motorLeftSlides.getCurrentPosition();
        double motorPower = error * Constants.SLIDE_MOTOR_KP;

        // slides not yet at target position
        if (Math.abs(error) > Constants.ROBOT_SLIDE_TOLERANCE_TICKS) {
            // slides going down - joystick
            if (error < 0 && error > Constants.MIN_SLIDE_ERROR_FULL_POWER) {
                motorLeftSlides.setPower(-0.3);
                motorRightSlides.setPower(-0.3);
            // slides going down - bumpers
            } else if (error < Constants.MIN_SLIDE_ERROR_FULL_POWER) {
                motorLeftSlides.setPower(-1.0);
                motorRightSlides.setPower(-1.0);
            // slides going up - proportional control
            } else {
                motorLeftSlides.setPower(motorPower);
                motorRightSlides.setPower(motorPower);
            }
        // slides at target position
        } else {
            motorLeftSlides.setPower(Constants.SLIDE_FEEDFORWARD);
            motorRightSlides.setPower(Constants.SLIDE_FEEDFORWARD);
        }
    }

    /**
     * uses the grabber camera to guide the robot so it can center on the top of the junction
     * @param pipeline the GrabberCameraPipeline being used by the grabber camera
     */
    public void centerJunctionTop(GrabberCameraPipeline pipeline) {
        double xOffset, yOffset;

        do {
            xOffset = pipeline.xPosition-Constants.CAMERA_CENTER_X;
            yOffset = Constants.CAMERA_CENTER_Y-pipeline.yPosition;

            // center the cone on the junction top
            if (pipeline.detected) {
                driveWithoutIMU(junctionTopPixelsMotorPower(xOffset), junctionTopPixelsMotorPower(yOffset), 0.0);
                telemetry.addData("x", pipeline.xPosition-Constants.CAMERA_CENTER_X);
                telemetry.addData("y", Constants.CAMERA_CENTER_Y-pipeline.yPosition);
                telemetry.addData("xMotorPower", junctionTopPixelsMotorPower(xOffset));
                telemetry.addData("yMotorPower", junctionTopPixelsMotorPower(yOffset));
                telemetry.update();
            } else {
                break;
            }

        // while the cone isn't centered over the junction
        } while (Math.abs(xOffset) > Constants.JUNCTION_TOP_TOLERANCE || Math.abs(yOffset) > Constants.JUNCTION_TOP_TOLERANCE);

        stopDriveMotors();
    }

    /**
     * uses the robot camera to guide the robot so it can drive forward while centering on the stack
     * @param pipeline the RobotCameraPipeline being used by the robot camera
     */
    public void centerConeStack(RobotCameraPipeline pipeline) {
        double xOffset, width, strafePower, turnPower;
        do {
            xOffset = Constants.CAMERA_CENTER_X-pipeline.xPosition;
            width = pipeline.width;

            // drive forward while centering on the cone stack if contour exists
            if (width == 0) {
                break;
            } else {
                strafePower = coneStackPixelsAndWidthToStrafingPower(xOffset, width);
                turnPower = 1.09*coneStackPixelsAndWidthToTurningPower(xOffset, width);
                driveWithIMU(strafePower, 1.35*coneStackWidthMotorPower(width), turnPower);
                telemetry.addData("xStrafingPower", strafePower);
                telemetry.addData("xTurningPower", turnPower);
                telemetry.addData("yMotorPower", coneStackWidthMotorPower(width));
                telemetry.addData("width", width);
                telemetry.update();
            }

            // while far enough that the cone stack doesn't fill the entire camera view
        } while (width < Constants.CONE_WIDTH);

        stopDriveMotors();
    }

    /**
     * calculates the motor power for the robot drivetrain based on the pixel offset from the junction top
     * @param pixelOffset how far junction top is from center of camera field of view in pixels
     * @return motor power for robot drivetrain
     */
    public double junctionTopPixelsMotorPower(double pixelOffset) {
        return Constants.JUNCTION_TOP_CENTERING_KP * pixelOffset;
    }

    /**
     * calculates the motor power for the robot drivetrain based on the width of the bounding box of the cone stack
     * @param coneStackWidth how wide the cone stack bounding box is in pixels
     * @return motor power for robot drivetrain
     */
    public double coneStackWidthMotorPower(double coneStackWidth) {
        return Constants.CONE_STACK_WIDTH_KP * coneStackWidth + Constants.CONE_STACK_CENTERING_MAX_SPEED;
    }

    /**
     * calculates the turning power for the robot drivetrain based on the pixel offset from the cone stack and the width of the cone stack
     * @param pixelOffset how far cone stack is horizontally from center of camera field of view in pixels
     * @param width how wide the cone stack is in the camera frame in pixels
     * @return turning power for robot drivetrain
     */
    public double coneStackPixelsAndWidthToTurningPower(double pixelOffset, double width) {
        return (Constants.CONE_STACK_CENTERING_PROPORTIONAL_KP * pixelOffset)/((1.0/Constants.TURNING_AUTHORITY_CONSTANT)*width);
    }

    /**
     * calculates the strafing power for the robot drivetrain based on the pixel offset from the cone stack and the width of the cone stack
     * @param pixelOffset how far cone stack is horizontally from center of camera field of view in pixels
     * @param width how wide the cone stack is in the camera frame in pixels
     * @return strafing power for robot drivetrain
     */
    public double coneStackPixelsAndWidthToStrafingPower(double pixelOffset, double width) {
        return (Constants.CONE_STACK_CENTERING_PROPORTIONAL_KP * pixelOffset) * ((1.0 / Constants.STRAFING_AUTHORITY_CONSTANT) * width);
    }

    /**
     * sets all drive motor powers to 0
     */
    public void stopDriveMotors() {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    /**
     * turns the LEDs green if it detects the top of a junction, otherwise they are rainbow colors
     */
    public void driveLEDs() {
        if (blinkinChassis != null) {
            if (grabberCameraPipeline.detected) {
                blinkinChassis.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                blinkinChassis.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
            }
        }
    }
}
