/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Reno;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Logging;

import java.util.ArrayList;
import java.util.List;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareRobot
{
    public enum  RobotAction
    {
        DRIVE_STRAIGHT,
        DRIVE_TURN,
        TURN,
        PICKUP_CONE,
        DROP_CONE,
        Localization
    }

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Public OpMode members. */
    public DcMotor  leftDriveFront   = null;
    public DcMotor  rightDriveFront  = null;
    public DcMotor  leftDriveBack = null;
    public DcMotor rightDriveBack = null;

    // definitions for slider motor, servo motor, and claw motors
    public DcMotor sliderMotor     = null;

    public Servo  gripperServo    = null;

    public DcMotor  leftArm     = null;
    //public Servo    leftClaw    = null;
    //public Servo    rightClaw   = null;

    //public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hardwareMap =  null;
    private ElapsedTime period  = new ElapsedTime();
    private String motorStatus, positionStatus;
    private BNO055IMU imu = null;

    private static final String VUFORIA_KEY =
            "AQFuTJD/////AAABmRQf1MrshUsMqfZFoea5LcBlQ79rSmu2Hsy0VUgvFAIXZWBCbzYDL8mVWF1K4NKNgTmlUGl" +
                    "mL6QrKaxd+oCP2VfTB5U+hgJ6KfZ5y2b6eV05AXaI/YWSyXQSdU/U7sKj+DuHMLbbK8wROJaJhoCoFz3" +
                    "HVWr9wknksiGASbHmm3B60ItfxXUm4oKrkBQGhVwOFfejn/Cq2tticMWg0Vv8fy/v4O2nf0vgLIHV5unW5" +
                    "CpuKiTfKAZqkfHgHYcB693Ebnkx3ZOBJy1Vv74xk1BSRRoR1HIjqQINhUWhws/yoHKsgq1z7COmzOzoh" +
                    "0WFj5jmLgjKDslJccFmcemOOYdejUA8u4SIARjiErm6Ow+kfvuU";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;
    private static final float meterPerInch        = 0.0254f;


    // Class Members
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null ;
    private WebcamName webcamName       = null;

    private VuforiaTrackable target;
    private OpenGLMatrix cameraLocation;

    private boolean targetVisible       = false;
    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;
    private VuforiaTrackableDefaultListener listener;
    List<VuforiaTrackable> allTrackables;

    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;
        Logging.log("init hardware");
        // Define and Initialize Motors
        leftDriveFront  = this.hardwareMap.get(DcMotor.class, "FrontLeft");
        rightDriveFront = this.hardwareMap.get(DcMotor.class, "FrontRight");
        leftDriveBack  = this.hardwareMap.get(DcMotor.class, "BackLeft");
        rightDriveBack = this.hardwareMap.get(DcMotor.class, "BackRight");
        sliderMotor = this.hardwareMap.get(DcMotor.class, "SliderMotor");  // for slider dc motor

        gripperServo = this.hardwareMap.get(Servo.class, "TestServo");  // for gripper servo motor

        this.setDriveForward();
        // Set all motors to zero power
        this.stop();

        gripperServo.setPosition(0.0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = this.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        this.setupVuforia();

    }

    public void turn(double power)
    {
        //telemetry.addData("Status", "ready to move backward for 2 seconds");
        //Moving backward
        //power = -0.2;

        this.leftDriveFront.setPower(power);
        this.leftDriveBack.setPower(power);
        this.rightDriveBack.setPower(power);
        this.rightDriveFront.setPower(power);
    }
    public void drive(double power)
    {
        // Moving forward
        //telemetry.addData("Status", "ready to move forward for 4 seconds");
        //power = 0.2;
        this.leftDriveFront.setPower(power);
        this.leftDriveBack.setPower(power);
        this.rightDriveBack.setPower(power);
        this.rightDriveFront.setPower(power);


    }

    public void stop()
    {
        Logging.log("stop all motors with o power");
        this.leftDriveFront.setPower(0);
        this.leftDriveBack.setPower(0);
        this.rightDriveBack.setPower(0);
        this.rightDriveFront.setPower(0);
        this.resetEncoder();
    }

    public void tankDrive(double leftPower, double rightPower) {

        this.leftDriveFront.setPower(leftPower);
        this.leftDriveBack.setPower(leftPower);
        this.rightDriveBack.setPower(rightPower);
        this.rightDriveFront.setPower(rightPower);
        this.setMotorStatus(leftPower, leftPower, rightPower, rightPower);
    }
    public void tankDrive(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower)
    {

        this.leftDriveFront.setPower(leftFrontPower);
        this.leftDriveBack.setPower(leftBackPower);
        this.rightDriveBack.setPower(rightBackPower);
        this.rightDriveFront.setPower(rightFrontPower);
        this.setMotorStatus(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    public void arcadeDrive(double drive, double rotate) {

        double maximum = Math.max(Math.abs(drive), Math.abs(rotate));
        double total = drive + rotate;
        double difference = drive - rotate;


        if(drive >= 0)
        {
            if (rotate >= 0)
            {
                this.tankDrive(maximum, difference);
            }
            else
            {
                this.tankDrive(total, maximum);
            }

        }
        else
        {
            if (rotate >= 0)
            {
                this.tankDrive(total, -1 * maximum);
            }
            else
            {
                this.tankDrive(-1 * maximum, difference);
            }
        }

    }
    public void drive(double drive, double turn) {
        double driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        double turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        //leftSpeed = Math.abs(leftSpeed);
        //rightSpeed = Math.abs(rightSpeed);

        Logging.log("driving ...");
        Logging.log(String.format("Power - LF %5.2f:LB %5.2f:RF %5.2f:RB %5.2f", leftSpeed, leftSpeed, rightSpeed, rightSpeed));

        leftDriveFront.setPower(leftSpeed);
        rightDriveFront.setPower(rightSpeed);
        leftDriveBack.setPower(leftSpeed);
        rightDriveBack.setPower(rightSpeed);
        this.setMotorStatus(leftSpeed,leftSpeed, rightSpeed, rightSpeed);
    }

    public void setDriveForward()
    {
        Logging.log("set drive forward");
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setDriveBackward()
    {
        Logging.log("set drive backward");
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
    }


    public void resetEncoder()
    {
        Logging.log("reset encoder into brake mode");
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void enableEncoder()
    {
        Logging.log("enable encoder");
        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void disableEncoder()
    {
        Logging.log("disable encoder");
        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetPosition(double distance)
    {
        Logging.log("set target position " + distance);
        int moveCounts = (int)(distance * COUNTS_PER_INCH);
        int leftFrontTarget = leftDriveFront.getCurrentPosition() + moveCounts;
        int rightFrontTarget = rightDriveFront.getCurrentPosition() + moveCounts;
        int leftBackTarget = leftDriveBack.getCurrentPosition() + moveCounts;
        int rightBackTarget = rightDriveBack.getCurrentPosition() + moveCounts;
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftDriveFront.setTargetPosition(leftFrontTarget);
        rightDriveFront.setTargetPosition(rightFrontTarget);
        leftDriveBack.setTargetPosition(leftBackTarget);
        rightDriveBack.setTargetPosition(rightBackTarget);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Logging.log(String.format("Target Pos - LF %d:LB %d:RF %d:RB %d", leftFrontTarget, rightFrontTarget, leftBackTarget, rightBackTarget));
    }
    public void setMotorPower(double leftFront, double rightFront, double leftBack, double rightBack){
        leftDriveFront.setPower(leftFront);
        leftDriveBack.setPower(leftBack);
        rightDriveBack.setPower(rightBack);
        rightDriveFront.setPower(rightFront);
    }

    public String getCurrentPosition() {

        return String.format("Current Pos - LF %d:LB %d:RF %d:RB %d", leftDriveFront.getCurrentPosition(), rightDriveFront.getCurrentPosition() , leftDriveBack.getCurrentPosition(), rightDriveBack.getCurrentPosition());
    }

    public void setMotorStatus(double leftFrontSpeed, double leftBackSpeed, double rightFrontSpeed, double rightBackSpeed)
    {
        motorStatus =  String.format("Power - LF %5.2f:LB %5.2f:RF %5.2f:RB %5.2f", leftFrontSpeed, leftBackSpeed, rightFrontSpeed, rightBackSpeed);
    }

    public String getMotorStatus()
    {
        return this.motorStatus;
    }
    public String getTargetPosition()
    {
        return this.positionStatus;
    }

    public boolean isBusyDriving()
    {
        return leftDriveFront.isBusy() && rightDriveFront.isBusy() && leftDriveBack.isBusy() && rightDriveBack.isBusy();
    }
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }

    public double getRawRoll() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(angles.secondAngle);
    }

    public double getRawPitch() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(angles.thirdAngle);
    }

    private void setupVuforia()
    {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        // Setup the target to be tracked
        identifyTarget(0, "Red Audience Wall",   -halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(1, "Red Rear Wall",        halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall",  -halfField,   oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(3, "Blue Rear Wall",       halfField,   oneAndHalfTile, mmTargetHeight, 90, 0, -90);

        target = allTrackables.get(0);
        target.setName("Red Audience Wall");


        target.setLocation(createMatrix((24 * meterPerInch), (24 * meterPerInch), 0, 90, 0, 90));

        // Set phone location on robot
        cameraLocation = createMatrix((10 * meterPerInch), (float)(-4.75 * meterPerInch), 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        //listener = (VuforiaTrackableDefaultListener) target.getListener();
        //listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocation);
        }
    }

    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public RobotLocation getRobotLocationOnField()
    {
        for (VuforiaTrackable trackable : allTrackables)
        {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
            {
                listener = ((VuforiaTrackableDefaultListener)trackable.getListener());
                OpenGLMatrix robotLocationTransform = listener.getUpdatedRobotLocation();
                OpenGLMatrix targetLocationTransform = listener.getVuforiaCameraFromTarget();
                OpenGLMatrix robotLocationOnField = listener.getFtcFieldFromRobot();
                OpenGLMatrix cameraLocationOnRobot = listener.getCameraLocationOnRobot(webcamName);
                OpenGLMatrix targetLocationOnField = target.getFtcFieldFromTarget();

                float[] coordinates = robotLocationTransform.getTranslation().getData();

                robotX = coordinates[0] * 1000;
                robotY = coordinates[1] * 1000;
                Orientation orientation = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                RobotLocation robotLocation = new RobotLocation(robotX / mmPerInch, robotY / mmPerInch, 0, orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle);

                Logging.log(robotLocation.toString());
                return robotLocation;
            }
        }
        Logging.log("Robot location cannot be identified.");
        return null;
    }
    VuforiaTrackable    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));

        return aTarget;
    }

}

