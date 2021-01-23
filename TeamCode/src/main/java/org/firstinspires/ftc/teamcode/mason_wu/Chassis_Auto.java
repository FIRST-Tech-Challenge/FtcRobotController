package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.io.File;
import java.util.List;
import java.util.Locale;

@Autonomous(name="Chassis Autonomous BLUE", group="4100")
public class Chassis_Auto extends LinearOpMode {
    // Declare OpMode members.
    private DcMotor Arm = null;
    private Servo Hand = null;
    private DcMotor Intake = null;
    private DcMotor Shooter = null;
    private Servo Spanker = null;
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

    BNO055IMU imu;
    final double HAND_CLOSE_POSITION = 0.9;
    final double HAND_OPEN_POSITION = 0.0;

    //Vuforia setup for vision
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            Robot4100Common.VUFORIA_LICENSE;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables.
        Hand = hardwareMap.get(Servo.class, "hand");
        Arm = hardwareMap.get(DcMotor.class, "arm");

        Intake = hardwareMap.get(DcMotor.class, "intake");
        Spanker = hardwareMap.get(Servo.class, "spanker");
        Shooter = hardwareMap.get(DcMotor.class, "shooter");

        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        Arm.setDirection(DcMotor.Direction.REVERSE);
        Hand.setPosition(HAND_CLOSE_POSITION);
        //armMotion(true, 0.8, 400);

        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Spanker.setPosition(0.85);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        telemetry.addData("Gyro Calibration Status", imu.getCalibrationStatus().toString());

        //Initializing vision
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            // tfod.setZoom(2.5, 1.78);
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            /*
            plan the motion of chassis here:
                available method: stopMotion(long timeInterval)
                                  driveStraight(double margin, double power, double timeInterval)
                                  drivePerpendicularly(boolean isLeft, double margin, double power, double timeInterval)
                                  rotateAtAngle (boolean isClockwise, double degree, double margin, double power)
                                  rotateToAngle (double targetAngle, double margin, double power)
             */

            String visionResult = null;
            ElapsedTime recogTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            while (opModeIsActive() && recogTime.milliseconds() <= 1500.0) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            visionResult = recognition.getLabel();
                        }
                        telemetry.update();
                    }
                }
            }
            if (tfod != null) {
                tfod.shutdown();
            }

            telemetry.addLine("Result from Object Detection:" + visionResult);
            telemetry.update();
            sleep(3000);
            //code below is for ZERO ring scenario
            if (visionResult == null) {
                driveStraight(true, 1.0, -0.6, 1050);
                stopMotion(100);
                rotateToAngle(90.0, 1.0, 0.2);
                stopMotion(100);
                driveStraight(true, 1.0, -0.6, 400);
                stopMotion(100);
                rotateToAngle(0.0, 1.0, 0.2);
                stopMotion(100);
                // need code for shooting here
                Shooter.setPower(1.0);
                sleep(1000);
                for (int i = 0; i < 3; i++) {
                    Spanker.setPosition(0.55);
                    sleep(500);
                    Spanker.setPosition(0.85);
                    sleep(500);
                }
                Shooter.setPower(0.0);
                sleep(100);
                rotateToAngle(90.0, 1.0, 0.2);
                sleep(100);
                driveStraight(true, 1.0, -0.6, 250);
                sleep(100);
                armMotion(false, 0.8, 300);
                sleep(250);
                handMotion(false);
                sleep(250);
                armMotion(true, 0.8, 400);
                sleep(250);
                handMotion(true);
                sleep(250);
                rotateToAngle(0, 1.0, 0.2);
                sleep(100);
                driveStraight(true, 1.0, -0.6, 250);
            }

            //code below is for SINGLE ring scenario
            else if (visionResult.toLowerCase().equals("single")) {
                // code below drives and positions for shooting
                driveStraight(true, 1.0, -0.6, 1050);
                stopMotion(100);
                rotateToAngle(90.0, 1.0, 0.2);
                stopMotion(100);
                driveStraight(true, 1.0, -0.6, 400);
                stopMotion(100);
                rotateToAngle(0.0, 1.0, 0.2);
                stopMotion(100);
                Shooter.setPower(1.0);
                sleep(1000);
                for (int i = 0; i < 3; i++) {
                    Spanker.setPosition(0.55);
                    sleep(500);
                    Spanker.setPosition(0.85);
                    sleep(500);
                }
                Shooter.setPower(0.0);
                // code below would drive forward to drop wobble
                driveStraight(true, 1.0, -0.6, 375);
                stopMotion(100);
                // drops wobble here
                armMotion(false, 0.8, 300);
                sleep(250);
                handMotion(false);
                sleep(250);
                armMotion(true, 0.8, 400);
                sleep(250);
                handMotion(true);
                sleep(250);
                driveStraight(true, 1.0, 0.6, 300);
            }

            //code below is for QUAD rings scenario
            else if (visionResult.toLowerCase().equals("quad")) {
                driveStraight(true, 1.0, -0.6, 1050);
                stopMotion(100);
                rotateToAngle(90.0, 1.0, 0.2);
                stopMotion(100);
                driveStraight(true, 1.0, -0.6, 400);
                stopMotion(100);
                rotateToAngle(0.0, 1.0, 0.2);
                stopMotion(100);
                Shooter.setPower(1.0);
                sleep(1000);
                for (int i = 0; i < 3; i++) {
                    Spanker.setPosition(0.55);
                    sleep(500);
                    Spanker.setPosition(0.85);
                    sleep(500);
                }
                Shooter.setPower(0.0);
                // code below would drive forward and turn to drop wobble
                driveStraight(true, 1.0, -0.6, 975);
                stopMotion(100);
                rotateToAngle(90.0, 1.0, 0.2);
                stopMotion(100);
                driveStraight(true, 1.0, -0.6, 200);
                stopMotion(100);
                // drops wobble here
                armMotion(false, 0.8, 300);
                sleep(250);
                handMotion(false);
                sleep(250);
                armMotion(true, 0.8, 400);
                sleep(250);
                handMotion(true);
                sleep(250);
                rotateToAngle(0.0, 1.0, 0.2);
                sleep(250);
                driveStraight(true, 1.0, 0.6, 525);

            } else {
                telemetry.addLine("Fail to recognize");
                telemetry.update();
                sleep(2000);
            }


        }
    }

    void stopMotion() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }

    void stopMotion(long timeInterval) {
        stopMotion();
        sleep(timeInterval);
    }

    double normalizeAngle(double angle) {
        double tempDeg = angle % 360;
        if (tempDeg >= 180) {
            tempDeg -= 360;
        } else if (tempDeg < -180) {
            tempDeg += 360;
        }
        return tempDeg;
    }

    double aquireHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        double tempHead = normalizeAngle(heading);
        telemetry.addData("Heading", tempHead);
        telemetry.update();
        sleep(20);
        return tempHead;
    }

    void displayVoltageEncoderValue() {
        try {
            double volt = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
            telemetry.addData("Motor Voltage: ", volt);
            telemetry.update();
        } catch (Exception e) {
            telemetry.addLine("Unable to find voltage value");
            telemetry.update();
        }
        try {
            double LFEnc = LF.getCurrentPosition();
            double RFEnc = RF.getCurrentPosition();
            double LBEnc = LB.getCurrentPosition();
            double RBEnc = RB.getCurrentPosition();
            telemetry.addData("LF Encoder Value: ", LFEnc);
            telemetry.addData("RF Encoder Value: ", RFEnc);
            telemetry.addData("LB Encoder Value: ", LBEnc);
            telemetry.addData("RB Encoder Value: ", RBEnc);
            telemetry.addData("Average Encoder Value: ", (LFEnc + LBEnc + RFEnc + RBEnc) / 4.0);
            telemetry.update();
        } catch (Exception e) {
            telemetry.addLine("Unable to find encoder value");
            telemetry.update();
        }
    }

    void driveStraight(boolean isForward, double margin, double power, double timeInterval) throws InterruptedException {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int straightFactor = -1;
        if (isForward) {
            straightFactor = 1;
        }
        double targetAngle = currentAngle;
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = straightFactor * power;
            LB_power = straightFactor * power;
            RF_power = straightFactor * power;
            RB_power = straightFactor * power;
            if (tempAngle < normalizeAngle(targetAngle - 1 * margin)) {
                RF_power += 0.1;
                RB_power += 0.1;
                LF_power -= 0.1;
                LB_power -= 0.1;
            } else if (tempAngle > normalizeAngle(targetAngle + (margin))) {
                RF_power -= 0.1;
                RB_power -= 0.1;
                LF_power += 0.1;
                LB_power += 0.1;
            }
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
            displayVoltageEncoderValue();
        }
        stopMotion();
    }

    void drivePerpendicularly(boolean isLeft, double margin, double power, double timeInterval) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int perpendicularFactor = -1;
        if (isLeft) {
            perpendicularFactor = 1;
        }
        double targetAngle = normalizeAngle(currentAngle + 90 * perpendicularFactor);
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = -1 * perpendicularFactor * power;
            LB_power = perpendicularFactor * power;
            RF_power = perpendicularFactor * power;
            RB_power = -1 * perpendicularFactor * power;
            if (tempAngle < normalizeAngle(targetAngle - 1 * margin)) {
                RF_power += perpendicularFactor * 0.1;
                RB_power -= perpendicularFactor * 0.1;
                LF_power += perpendicularFactor * 0.1;
                LB_power -= perpendicularFactor * 0.1;
            } else if (tempAngle > normalizeAngle(targetAngle + (margin))) {
                RF_power -= perpendicularFactor * 0.1;
                RB_power += perpendicularFactor * 0.1;
                LF_power -= perpendicularFactor * 0.1;
                LB_power += perpendicularFactor * 0.1;
            }
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
        stopMotion();
    }


    //make a turn that based on the current heading in a certain direction and angle
    void rotateAtAngle(boolean isClockwise, double degree, double margin, double power) {
        int angleFactor = -1;
        if (!isClockwise) {
            angleFactor = 1;
        }
        final double currentAngle = aquireHeading();
        double targetAngle = normalizeAngle(currentAngle + degree * angleFactor);
        rotateToAngle(targetAngle, margin, power);
        stopMotion();
    }

    //make a turn TO a certain angle
    void rotateToAngle(double targetAngle, double margin, double power) {
        int angleFactor = 0;
        final double currentAngle = aquireHeading();
        if (currentAngle - targetAngle > 0) {
            if (currentAngle - targetAngle < 180) {
                //cw
                angleFactor = -1;
            } else {
                //ccw
                angleFactor = 1;
            }
        } else {
            if (targetAngle - currentAngle < 180) {
                //ccw
                angleFactor = 1;
            } else {
                //cw
                angleFactor = -1;
            }
        }
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        double tempAngle = currentAngle;
        while (!((tempAngle < targetAngle + margin) && (tempAngle > targetAngle - margin))) {
            tempAngle = aquireHeading();
            RF_power = angleFactor * power;
            RB_power = angleFactor * power;
            LF_power = -1 * angleFactor * power;
            LB_power = -1 * angleFactor * power;
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
        stopMotion();
    }

    boolean armMotion(boolean moveUp, double power, double timeInterval) {
        ElapsedTime armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        int upFactor = -1;
        if (!moveUp) {
            upFactor = 1;
        }
        while (armTime.milliseconds() <= timeInterval) {
            if (armTime.milliseconds() <= timeInterval - 300) {
                power -= upFactor * 0.02;
            }
            Arm.setPower(Math.abs(power) * (upFactor));
            sleep(20);
        }
        Arm.setPower(0);
        return true;
    }

    boolean handMotion(boolean close) {
        if (close) {
            Hand.setPosition(HAND_CLOSE_POSITION);
        } else {
            Hand.setPosition(HAND_OPEN_POSITION);
        }
        telemetry.addData("Hand Position:", Hand.getPosition());
        telemetry.update();
        return true;
    }

    //Vision
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.4f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }


}