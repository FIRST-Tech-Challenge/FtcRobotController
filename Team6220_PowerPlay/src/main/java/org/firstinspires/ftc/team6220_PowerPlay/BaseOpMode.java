package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.testclasses.ConeAndJunctionDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.team6220_PowerPlay.testclasses.ConeDetectionPipeline;

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
        PwmControl.PwmRange maxRange = new PwmControl.PwmRange(500, 2500, 20000);
        servoGrabber = (ServoImplEx) hardwareMap.servo.get("servoGrabber");
        servoGrabber.setPwmRange(maxRange);

        // initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        sleep(3000);

        startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        originalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        servoGrabber.setPosition(Constants.GRABBER_INITIALIZE_POSITION);
    }

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
            double angleError = originalAngle  - currentAngle;

            // flip to inverse of angles above 180 / below -180 (to prevent infinity-rotate bug)
            // to make sure to use the shorter angle
            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }

            // apply a constant to turn the angle into a turn speed
            tPower = -Constants.HEADING_CORRECTION_KP * angleError;
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
            if (error < 0 && error > -200) {
                motorLeftSlides.setPower(-0.3);
                motorRightSlides.setPower(-0.3);
            // slides going down - bumpers
            } else if (error < -200) {
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

    // takes a detection pipeline and temporarily takes control of the robot movement
    // until the robot has centered by reading the pipeline fields
    public void centerJunctionTop(ConeDetectionPipeline pipeline) {
        double xOffset, yOffset;
        do {
            xOffset = pipeline.Xpos - Constants.CAMERA_CENTER_X;
            yOffset = Constants.CAMERA_CENTER_Y - pipeline.Ypos;

            // convert the offsets to motor powers to drive with
            driveWithIMU(offsetToMotorPower(xOffset), offsetToMotorPower(yOffset), 0);

            // while either of the offsets are still too large
        } while (Math.abs(xOffset) > Constants.AUTOCENTER_ACCURACY || Math.abs(yOffset) > Constants.AUTOCENTER_ACCURACY);
    }

    // scales the offset from pixels to a motor power, stopping at +1/-1,
    // and slopes in towards 0 power after a certain point when nearing 0 offset
    public double offsetToMotorPower(double offsetPixels) {
        return (-0.1 * offsetPixels) / (Math.abs(0.25 * offsetPixels) + 15.0);
    }
}
