package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ArmServos;
import org.firstinspires.ftc.teamcode.Functions.CameraDetector;
import org.firstinspires.ftc.teamcode.Functions.CarouselMotor;
import org.firstinspires.ftc.teamcode.Functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.EncoderMove;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.MoveAutocorrect2;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.Vacuum;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;
import org.firstinspires.ftc.teamcode.Functions.CameraDetector;

import java.util.List;
import java.util.List;

@Disabled
public class Webcam2 extends LinearOpMode {

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack,vaccumRight,vaccumLeft, carouselMotor, leftMotorEncoder, rightMotorEncoder, leftMotorBackEncoder, rightMotorBackEncoder, armMotorChain;
    private DcMotorEx armMotorLeft, armMotorRight;
    //private Servo collectorServo;
    private CRServo collectorCr;
    private Move move;
    private Rotate rotate;
    private Arm arm;
    private Vacuum vaccum;
    private RotationDetector rotationDetector;
    private ArmEncoder armEncoder;
    private CarouselMotor _carouselMotor;
    private Collector collector;
    public VoltageReader voltageReader;
    private Servo L1Servo;
    private Servo L2Servo;
    public MoveAutocorrect2 AutoCorrection;
    private ArmServos armServos;
    private EncoderMove encoderMove;
    //private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    //private static final String TFOD_MODEL_ASSET = "tse_nou.tflite";
    //private static final String[] LABELS = {};
//            "Ball",
//            "Cube",
//            "Duck",



    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    public String label = null;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AeK2PvT/////AAABmTTCnhmbVUgFg93ZCRG8vUF1RRs5KbLEL7ILBtDXxAX3VJNJBBvkAEB2xBQ6yqr9yOlQWKwHR9mAKBkwOX" +
                    "7SUJJwHIwNimUwhfIpyQ6bsANOY67gAgPxBNXO2TGp7WXAAi+h/JVHjTPBJVMMmah9JURERd1w50biR4+6Mltk44izNVk" +
                    "JuH5RVuxyX2BpK7BlCDsus8/o7280n6CaQeJwqkZwW7WVuzdzyi0JdZL5nmgCHOI65lNQrKKu9ldVA4NBabfk6Lj5kSvd40u" +
                    "e4fUJRzPxPuiSoxgpJ5PFpsmuCPyJN5EOO1EITRSqXvtHZfYChrxIQKjtut+ihbbW8f6y3KeQqpRq5WbQxuQ6cPbuBhJf ";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        vaccumLeft = hardwareMap.dcMotor.get("VL");
        L1Servo = hardwareMap.servo.get("L1S");
        L2Servo = hardwareMap.servo.get("L2S");
        armMotorRight = hardwareMap.get(DcMotorEx.class, "AMR");
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "AML");
        armMotorChain = hardwareMap.get(DcMotor.class, "AMC");
        arm = new Arm(armMotorLeft, armMotorRight, armMotorChain);
        //
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        vaccum = new Vacuum(vaccumLeft);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        //_carouselMotor = new CarouselMotor(carouselMotor);
        collectorCr = hardwareMap.crservo.get("CR");
        collector = new Collector(collectorCr);
        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        armServos = new ArmServos(L1Servo, L2Servo);
        encoderMove = new EncoderMove(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        AutoCorrection = new MoveAutocorrect2(rotationDetector,move,rotate);
//        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).

            tfod.setZoom(1.8, 16.0/6.0);

        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

//
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
////        leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        boolean isDuckDetected = false;
        int number = 0;
        if (tfod!=null) {
            while (isDuckDetected==false) {

                    telemetry.addData("number=", number);
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
                            i++;
                            telemetry.update();
                            label=recognition.getLabel();

                            if(label.equals("Duck"))
                            {
                                addData(recognition.getLeft(), recognition.getRight(), recognition.getTop(), recognition.getBottom());
                                isDuckDetected = true;
                                break;
                            }
                        }
                        number++;

                        if(number>=45)

                        {
                            isDuckDetected=true;
                            break;
                        }
                        telemetry.update();
                    }
            }
        }
        if(label!=null)
        {
            if(duckValues.right<300)
            {

                telemetry.addData("TSE location:", "Left");
                telemetry.update();
                Zona1_incercare();
            }
            else if(duckValues.left>330 && duckValues.right<485)
            {
                telemetry.addData("TSE location:", "Middle");
                telemetry.update();
                Zona2_incercare();
            }
            else if(duckValues.left>500)
            {
                telemetry.addData("TSE location:", "Right");
                telemetry.update();
                Zona3_incercare();

                telemetry.addData("Duck location:", "Left");
                telemetry.update();
            }
            else if(duckValues.left>340 && duckValues.right<470)
            {
                telemetry.addData("Duck location:", "Middle");
                telemetry.update();
            }
            else if(duckValues.left>520)
            {
                telemetry.addData("Ducl location:", "Right");
                telemetry.update();

            }
        }


    }

    private void Zona1_incercare()
    {
        vaccumLeft.setPower(-0.5);

        move.MoveFull(4);
        sleep(voltageReader.GetWaitTime(10, 2));
        move.MoveStop();

        int angle = 55;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(500);

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(60, 1));
        move.MoveStop();

        arm.Start(-1);
        sleep(2000);
        armServos.Level1Up();
        sleep(1000);
        arm.Stop();
        sleep(1000);

        move.MoveRaw(2, 0.45);
        sleep(voltageReader.GetWaitTime(34, 1));
        move.MoveStop();
        sleep(500);

        vaccumLeft.setPower(0.5);
        sleep(800);

        move.MoveRaw(1, 0.5);
        sleep(voltageReader.GetWaitTime(35, 1));
        move.MoveStop();

        arm.Start(-1);
        sleep(1000);
        armServos.Level1Down();
        sleep(1000);
        arm.Stop();
        sleep(1000);

        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(30 ,3));
        rotate.RotateStop();
        sleep(1000);

        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(100, 1));
        move.MoveStop();
        sleep(800);

        angle = 35;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(800);

        move.MoveRaw(1, 0.5);
        sleep(voltageReader.GetWaitTime(6, 1));
        move.MoveStop();

        collector.Start();
        sleep(3700);
        collector.Stop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(20, 1));
        move.MoveStop();

        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(28,3));
        rotate.RotateStop();
        sleep(1000);

        vaccumLeft.setPower(-0.5);
        sleep(500);

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(50,1));

        move.MoveFull(3);
        sleep(voltageReader.GetWaitTime(25, 2));
        move.MoveStop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(165,1));
    }
    private void Zona2_incercare()
    {
        vaccumLeft.setPower(-0.5);

        move.MoveFull(4);
        sleep(voltageReader.GetWaitTime(10, 2));
        move.MoveStop();

        int angle = 55;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(500);

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(60, 1));
        move.MoveStop();

        arm.Start(-1);
        sleep(1400);
        armServos.Level2Up();
        sleep(1000);
        arm.Stop();
        sleep(1000);

        move.MoveRaw(2, 0.45);
        sleep(voltageReader.GetWaitTime(34, 1));
        move.MoveStop();
        sleep(500);

        vaccumLeft.setPower(0.5);
        sleep(800);

        move.MoveRaw(1, 0.5);
        sleep(voltageReader.GetWaitTime(35, 1));
        move.MoveStop();

        arm.Start(-1);
        sleep(1000);
        armServos.Level2Down();
        sleep(1000);
        arm.Stop();
        sleep(1000);

        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(30 ,3));
        rotate.RotateStop();
        sleep(1000);

        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(100, 1));
        move.MoveStop();
        sleep(800);

        angle = 40;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(800);

        move.MoveRaw(1, 0.5);
        sleep(voltageReader.GetWaitTime(6, 1));
        move.MoveStop();

        collector.Start();
        sleep(3700);
        collector.Stop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(20, 1));
        move.MoveStop();

        sleep(500);
        vaccumLeft.setPower(-0.5);
        sleep(500);

        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(28,3));
        rotate.RotateStop();
        sleep(1000);

        vaccumLeft.setPower(-0.5);
        sleep(500);

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(50,1));

        move.MoveFull(3);
        sleep(voltageReader.GetWaitTime(25, 2));
        move.MoveStop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(165,1));
    }
    private void Zona3_incercare()
    {
        vaccumLeft.setPower(-0.5);

        move.MoveFull(4);
        sleep(voltageReader.GetWaitTime(10, 2));
        move.MoveStop();

        int angle = 55;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(500);

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(55, 1));
        move.MoveStop();

        arm.Start(-1);
        sleep(1600);

        move.MoveRaw(2, 0.45);
        sleep(voltageReader.GetWaitTime(34, 1));
        move.MoveStop();
        sleep(500);

        vaccumLeft.setPower(0.5);
        sleep(800);

        move.MoveRaw(1, 0.5);
        sleep(voltageReader.GetWaitTime(35, 1));
        move.MoveStop();

        sleep(800);
        arm.Stop();
        sleep(1000);

        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(25 ,3));
        rotate.RotateStop();
        sleep(1000);

        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(100, 1));
        move.MoveStop();
        sleep(800);

        angle = 40;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(800);

        move.MoveRaw(1, 0.5);
        sleep(voltageReader.GetWaitTime(6, 1));
        move.MoveStop();

        collector.Start();
        sleep(3700);
        collector.Stop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(20, 1));
        move.MoveStop();

        sleep(500);
        vaccumLeft.setPower(-0.5);
        sleep(500);

        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(28,3));
        rotate.RotateStop();
        sleep(1000);

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(50,1));

        move.MoveFull(3);
        sleep(voltageReader.GetWaitTime(25, 2));
        move.MoveStop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(172,1));
    }
    private void Zona1_2()
    {
        vaccumLeft.setPower(-0.5);

        move.MoveFull(4);
        sleep(voltageReader.GetWaitTime(8, 2));
        move.MoveStop();

        move.MoveRaw(1, 0.7);
        sleep(voltageReader.GetWaitTime(50, 1));
        move.MoveStop();
        sleep(500);

        int angle = 35;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(500);

        move.MoveRaw(1, 0.5);
        sleep(voltageReader.GetWaitTime(6, 1));
        move.MoveStop();

        collector.Start();
        sleep(3700);
        collector.Stop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(85, 1));
        move.MoveStop();

        arm.Start(-1);
        sleep(1600);

        move.MoveRaw(2, 0.5);
        sleep(voltageReader.GetWaitTime(43, 1));
        move.MoveStop();
        sleep(500);
        vaccumLeft.setPower(0.5);
        sleep(800);


        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(25, 1));
        move.MoveStop();
        sleep(800);

        arm.Stop();
        sleep(1000);

        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(32,3));
        rotate.RotateStop();
        sleep(1000);

        vaccumLeft.setPower(-0.5);
        sleep(1000);

        move.MoveFull(3);
        sleep(voltageReader.GetWaitTime(42, 2));
        move.MoveStop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(174,1));
    }


    private void Zona2()
    {
        vaccumLeft.setPower(-0.5);

        move.MoveFull(4);
        sleep(voltageReader.GetWaitTime(8, 2));
        move.MoveStop();

        move.MoveRaw(1, 0.7);
        sleep(voltageReader.GetWaitTime(50, 1));
        move.MoveStop();
        sleep(500);

        int angle = 30;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(500);

        move.MoveRaw(1, 0.5);
        sleep(voltageReader.GetWaitTime(6, 1));
        move.MoveStop();

        collector.Start();
        sleep(3700);
        collector.Stop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(85, 1));
        move.MoveStop();

        arm.Start(-1);
        sleep(1500);
        armServos.Level1Down();
        sleep(500);
        arm.Stop();
        sleep(1000);

        rotate.RotateFull(2);
        sleep(voltageReader.GetWaitTime(4,3));
        rotate.RotateStop();

        move.MoveRaw(2, 0.5);
        sleep(voltageReader.GetWaitTime(42, 1));
        move.MoveStop();
        sleep(500);
        vaccumLeft.setPower(0.5);
        sleep(800);

        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(25, 1));
        move.MoveStop();

        arm.Start(-1);
        sleep(1000);
        armServos.Level1Up();
        sleep(1000);
        arm.Stop();
        sleep(1000);

        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(32,3));
        rotate.RotateStop();
        sleep(1000);

        vaccumLeft.setPower(-0.5);
        sleep(1000);

        move.MoveFull(3);
        sleep(voltageReader.GetWaitTime(42, 2));
        move.MoveStop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(175,1));
    }
    private void Zona3()
    {
        vaccumLeft.setPower(-0.5);

        move.MoveFull(4);
        sleep(voltageReader.GetWaitTime(8, 2));
        move.MoveStop();

        move.MoveRaw(1, 0.7);
        sleep(voltageReader.GetWaitTime(50, 1));
        move.MoveStop();
        sleep(500);

        int angle = 30;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(500);

        move.MoveRaw(1, 0.5);
        sleep(voltageReader.GetWaitTime(6, 1));
        move.MoveStop();

        collector.Start();
        sleep(3700);
        collector.Stop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(85, 1));
        move.MoveStop();

        arm.Start(-1);
        sleep(1600);
        rotate.RotateFull(2);
        sleep(voltageReader.GetWaitTime(4,3));
        rotate.RotateStop();
        move.MoveRaw(2, 0.7);
        sleep(voltageReader.GetWaitTime(32, 1));
        move.MoveStop();
        sleep(1000);
        vaccumLeft.setPower(0.5);
        sleep(800);

        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(25, 1));
        move.MoveStop();

        arm.Stop();
        sleep(1000);

        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(32,3));
        rotate.RotateStop();
        sleep(1000);

        vaccumLeft.setPower(-0.5);
        sleep(1000);

        move.MoveFull(3);
        sleep(voltageReader.GetWaitTime(42, 2));
        move.MoveStop();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(175,1));
    }
    private void Zona1_encodere()
    {
        vaccumLeft.setPower(-0.5);

        int centi = 35;
        while(opModeIsActive() && encoderMove.driveSlideLeft( 0.5)){

            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*centi)/33))){
//                encoderMove.reset();
                break;
            }
        }
        sleep(500);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centi = 40;
        while(opModeIsActive() && encoderMove.driveBack( 0.5)){

            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*centi)/33))){
                encoderMove.reset();
                break;
            }
        }
        sleep(500);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int rotateCenti = 18; // unghi
        while(opModeIsActive() && encoderMove.driveRotateLeft( 0.5)){

            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*rotateCenti)/33))){
                encoderMove.reset();
                break;
            }
        }
        sleep(800);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centi = 21;
        while(opModeIsActive() && encoderMove.driveBack( 0.5)){

            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*centi)/33))){
                encoderMove.reset();
                break;
            }
        }
        sleep(500);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        collector.Start();
        sleep(3700);
        collector.Stop();

//        centi = 20;
//        while(opModeIsActive() && encoderMove.drive( 0.7)){
//
//            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*centi)/33))){
//                encoderMove.reset();
//                break;
//            }
//        }
//        sleep(500);
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        rotateCenti = 5; // unghi
//        while(opModeIsActive() && encoderMove.driveRotateRight( 0.5)){
//
//            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*rotateCenti)/33))){
//                encoderMove.reset();
//                break;
//            }
//        }
//        sleep(800);
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centi = 75;
        while(opModeIsActive() && encoderMove.drive( 0.5)){

            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*centi)/33))){
                encoderMove.reset();
                break;
            }
        }
        sleep(500);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.Start(-1);
        sleep(1500);
        armServos.Level2Down();
        sleep(500);
        arm.Stop();
        sleep(1000);

        centi = 33;
        while(opModeIsActive() && encoderMove.drive( 0.5)){

            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*centi)/33))){
                encoderMove.reset();
                break;
            }
        }
        sleep(1000);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vaccumLeft.setPower(0.5);
        sleep(800);

        centi = 30;
        while(opModeIsActive() && encoderMove.driveBack( 0.5)){

            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*centi)/33))){
                encoderMove.reset();
                break;
            }
        }
        sleep(500);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.Start(-1);
        sleep(1000);
        armServos.Level2Up();
        sleep(1000);
        arm.Stop();
        sleep(1000);

        rotateCenti = 15;
        while(opModeIsActive() && encoderMove.driveRotateRight( 0.5)){

            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*rotateCenti)/33))){
                encoderMove.reset();
                break;
            }
        }
        sleep(1000);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vaccumLeft.setPower(-0.5);
        sleep(1000);

        centi = 60;
        while(opModeIsActive() && encoderMove.driveSlideRight( 0.5)){

            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*centi)/33))){
                encoderMove.reset();
                break;
            }
        }
        sleep(1000);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centi = 200;
        while(opModeIsActive() && encoderMove.drive( 0.5)){

            if(encoderMove.leftPos== (encoderMove.rightTarget1*((15*centi)/33))){
                encoderMove.reset();
                break;
            }
        }
        sleep(500);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    class data {
        float left;
        float right;
        float top;
        float bottom;
    }
    data duckValues = new data();

    int InitialAngle = 0;
    long Time;
    int Distance = 100;
    private void Autocorrect() {
        AutoCorrection.GivenAngle(InitialAngle);
        Time = 50;
        //Time = voltageReader.GetWaitTime(Distance, 1);
        while (Time > 0) {
            move.MoveFull(1);
            AutoCorrection.MoveFull(1);
            Time -= 1;
            telemetry.addData("Timpul ", Time);
            telemetry.update();
        }
        move.MoveStop();
    }


    void addData(float _left, float _right, float _top, float _bottom) {
        duckValues.left = _left;
        duckValues.right = _right;
        duckValues.top = _top;
        duckValues.bottom = _bottom;
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

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

        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
