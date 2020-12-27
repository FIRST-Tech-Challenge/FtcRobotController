package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drivebase.GyroSensor;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDrivebase;

import java.util.List;

@Autonomous(name = "Ring Auto Test", group = "Linear Opmode")
//@Disabled
public class RingAutoTest extends LinearOpMode {

    /* Declare OpMode members. */
    private MecanumDrivebase mecanumDrivebase = new MecanumDrivebase();
    private GyroSensor gyroSensor = new GyroSensor();

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: goBILDA Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    ElapsedTime Timer;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            " Ac/2h37/////AAABmbXAvaZQqkPSlZv4583jp15xBpCuzySKMfid1ppM+8fZbZsGd93ri87TKmjKKCYA64DjBiSRboJvg0eldCw/QzbXtH/gNzdbd90bD226N+MA3p3b4CH+C8Pe+Q2SPV5d4e23K514g/DZGu5JEHHH5kl1guWLfc485PCIGE/wlhIprwSQmGM535rO6oif8Dka9K6zFPkiiSvsj4SoTdVJ9EMPnSYT1LNRUtcWWyN0aCVFJ2cmU2lCAtvS6t7GACGTQAbq+vURBnS0BLwkqgebDbvPPM6y4LOG904dFosYxQsSJw51CCTDNLXlunkQcEzp8DSjH79jiTb6BMwGtpRbFhyGrtSq+ugYlE6uf+C7V913 ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void runOpMode() {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setClippingMargins(0,250,0,0);
            //                      L T R B
        }

        mecanumDrivebase.initialize(this);
        // Wait for the start button

        mecanumDrivebase.startControl();
        //gyroSensor.startControl();

        //BNO055IMU.Parameters IMU_Parameters;
        //float Yaw_Angle;

        //IMU_Parameters = new BNO055IMU.Parameters();
        //IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        //gyroSensor.initialize(this);
        telemetry.addData("Status", "Robot initialized" + ".");
        telemetry.update();
        CameraDevice.getInstance().setFlashTorchMode(true);

        waitForStart();

        if (opModeIsActive()) {

            mecanumDrivebase.resetEncoders();
            mecanumDrivebase.FL.setDirection(DcMotorSimple.Direction.REVERSE);
            mecanumDrivebase.BL.setDirection(DcMotorSimple.Direction.REVERSE);
            mecanumDrivebase.FR.setDirection(DcMotorSimple.Direction.FORWARD);
            mecanumDrivebase.BR.setDirection(DcMotorSimple.Direction.FORWARD);
            mecanumDrivebase.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanumDrivebase.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanumDrivebase.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecanumDrivebase.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            float encoderFR = mecanumDrivebase.FR.getCurrentPosition();
            float encoderFL = mecanumDrivebase.FL.getCurrentPosition();
            float encoderBR = mecanumDrivebase.BR.getCurrentPosition();
            float encoderBL = mecanumDrivebase.BL.getCurrentPosition();

            double avgDistance = ((encoderBL + encoderBR + encoderFL + encoderFR) / 4) / COUNTS_PER_INCH;
            double time = 0.0;
            boolean ringsRecognized = false;
            double ringType = -1; //-1 = unsure, 0 = no rings, 1 = single ring, 4 = quad (4) rings

            Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            Timer.reset();

            while (ringsRecognized != true && time < 2525) {
                time = Timer.milliseconds();

                if (time > 2500) {
                    ringType = 0; //No ring seen
                }

                if ((time > 1500) && (time < 2525)) {
                    mecanumDrivebase.stop();
                } else {
                    mecanumDrivebase.FR.setPower(0.15);
                    mecanumDrivebase.FL.setPower(0.15);
                    mecanumDrivebase.BR.setPower(0.15);
                    mecanumDrivebase.BL.setPower(0.15);
                }

                encoderFR = mecanumDrivebase.FR.getCurrentPosition();
                encoderFL = mecanumDrivebase.FL.getCurrentPosition();
                encoderBR = mecanumDrivebase.BR.getCurrentPosition();
                encoderBL = mecanumDrivebase.BL.getCurrentPosition();
                avgDistance = ((encoderBL + encoderBR + encoderFL + encoderFR) / 4) / COUNTS_PER_INCH;

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        if (recognition.getLabel() == "Single") {
                            ringType = 1;
                        } else if (recognition.getLabel() == "Quad") {
                            ringType = 4;
                        }
                        ringsRecognized = true;
                    }
                }


                telemetry.addData("Time (Milliseconds)", time);
                telemetry.addData("Avg Distance (Inches)", avgDistance);
                telemetry.update();
            }

            mecanumDrivebase.stop();

            telemetry.addData("Time (Milliseconds)", time);
            telemetry.addData("Avg Distance (Inches)", avgDistance);
            telemetry.addData("Number of Rings Seen", ringType);
            telemetry.update();
            sleep(5000);
            }
        }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

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
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}