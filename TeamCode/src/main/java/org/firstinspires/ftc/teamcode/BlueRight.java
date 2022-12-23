package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.apriltag.AprilTagDetection;
import java.util.ArrayList;

@Autonomous
public class BlueRight extends LinearOpMode {
    OpenCvCamera webcam;
    Pipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    int location;

    AprilTagDetection tagOfInterest = null;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor Spin;
    DcMotor Crane;
    CRServo Left;


    BNO055IMU imu;
    Orientation angles;



    @Override
    public void runOpMode() {
        initGyro();
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new Pipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null) {
            location=0;
        } else if (tagOfInterest.id == LEFT) {
            location=1;
        } else if (tagOfInterest.id == MIDDLE) {
            location=2;
        } else {
            location=3;
        }


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        Left = hardwareMap.get(CRServo.class, "Lefts");
        Spin = hardwareMap.get(DcMotor.class, "Spin");
        Crane = hardwareMap.get(DcMotor.class, "Crane");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        if (opModeIsActive()) {

            crane(-1, 400);
            move(.5, 200);
            strafeLeft(1, 1000);
            gyroTurning(0);
            sleep(2000);
            move(1, 2000);
            gyroTurning(-90);
            move(1, 600);
            crane(-1, 3600);

            switch (location){
                case 0:
                    //
                    break;
                case 1:
                    //
                    break;
                case 2:
                    //
                    break;

                case 3:
                    //
                    break;
            }
        }
    }


        public void initGyro () {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            sleep(250);
        }
        void tagToTelemetry (AprilTagDetection detection)
        {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }

        public boolean gyroTurning ( double targetAngle){
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            boolean foundAngle;
            foundAngle = false;
            while (!foundAngle) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double currentAngle = angles.firstAngle;
                telemetry.addData("Angle", currentAngle);
                telemetry.addData("targetangle", targetAngle);
                telemetry.update();
                if (angles.firstAngle >= targetAngle - 0.1 && angles.firstAngle <= targetAngle + 0.1) {
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    foundAngle = true;
                    sleep(1000);
                    break;

                } else if (angles.firstAngle >= targetAngle + 0.5) {
                    if (angles.firstAngle <= targetAngle - 5) {
                        frontLeft.setPower(0.2);
                        frontRight.setPower(-0.2);
                        backLeft.setPower(0.2);
                        backRight.setPower(-0.2);
                        foundAngle = false;
                    } else {
                        frontLeft.setPower(-0.2);
                        frontRight.setPower(0.2);
                        backLeft.setPower(-0.2);
                        backRight.setPower(0.2);
                        foundAngle = false;
                    }
                } else if (angles.firstAngle <= targetAngle - 0.5) {
                    if (angles.firstAngle >= targetAngle + 5) {
                        frontLeft.setPower(-0.2);
                        frontRight.setPower(0.2);
                        backLeft.setPower(-0.2);
                        backRight.setPower(0.2);
                        foundAngle = false;
                    } else {
                        frontLeft.setPower(.2);
                        frontRight.setPower(-.2);
                        backLeft.setPower(.2);
                        backRight.setPower(-.2);
                        foundAngle = false;
                    }
                }
            }
            return foundAngle;
        }

        public void stopMotors () throws InterruptedException {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        public void move ( double power, int position){
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontRight.setTargetPosition(-position);
            frontLeft.setTargetPosition(-position);
            backRight.setTargetPosition(-position);
            backLeft.setTargetPosition(-position);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);

            while (frontLeft.isBusy() && opModeIsActive()) {

            }

        }
        public void craneinput () {
            crane(1, -550);
            move(0.2, 350);
            //Crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Crane.setTargetPosition(0);
            Crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Crane.setPower(1);
            while (Crane.isBusy() && opModeIsActive()) {
            }
        }

        public void strafeLeft ( double power, int position) {
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontRight.setTargetPosition(-position);
            frontLeft.setTargetPosition(position);
            backRight.setTargetPosition(position);
            backLeft.setTargetPosition(-position);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);

            while (frontLeft.isBusy() && opModeIsActive()) {

            }

        }

        public void strafeRight ( double power, int position) throws InterruptedException {
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontRight.setTargetPosition(position);
            frontLeft.setTargetPosition(-position);
            backRight.setTargetPosition(-position);
            backLeft.setTargetPosition(position);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);

            while (frontLeft.isBusy() && opModeIsActive()) {

            }
        }

        public void intake ( int direction, long time) throws InterruptedException {
            Left.setPower(direction * 1);
            sleep(time);
            Left.setPower(0);

        }

    /*public void moveandspin(double power, int moveposition, int spinposition) {
        move(power, moveposition);
        spin(spinposition);
        while (Spin.isBusy()) {

        }
    }*/

        public void spin ( int SpinPosition, int CranePosition){
            Spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Spin.setTargetPosition(SpinPosition);
            Crane.setTargetPosition(CranePosition);
            Spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Spin.setPower(1);
            Crane.setPower(1);
            while (Spin.isBusy() && opModeIsActive()) {

            }
        }

        public void crane( double power, int time){
            Crane.setPower(power);
            sleep(time);
            Crane.setPower(0);

        }
    }


