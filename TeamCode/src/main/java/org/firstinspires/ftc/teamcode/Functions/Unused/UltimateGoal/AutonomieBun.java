package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem.PositionCalculator;
import org.firstinspires.ftc.teamcode.Functions.Rotate;

import java.io.IOException;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Functions.MV.MVTurnTowardsPoint;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;

import java.util.List;


//import org.firstinspires.ftc.teamcode.Autonomie.WebcamDetector;


/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutonomieBunMnoa", group = "Concept")
@Disabled
public class AutonomieBun extends LinearOpMode {
    private boolean ok=false;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public String label=null;
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack, vaccumMotor, left_pistol,right_pistol;
    private DcMotor spagheteMotor, arm_motor;
    public ModernRoboticsI2cRangeSensor rangeFinder = null;
    private Servo push_ring, hold_wobble;
    public Move move;
    public Rotate rotate;
    public Aspirator aspirator;
    public Pistol pistol;
    public VoltageReader voltageReader;
    public HoldWobble holdwobble;
    public RotationDetector rotationDetector;
    public Tragaci tragaci;
    public MVTurnTowardsPoint MVTurnTowardsPoint;

    //  public WebcamDetector webcamDetector;
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
            " AeK2PvT/////AAABmTTCnhmbVUgFg93ZCRG8vUF1RRs5KbLEL7ILBtDXxAX3VJNJBBvkAEB2xBQ6yqr9yOlQWKwHR9mAKBkwOX7SUJJwHIwNimUwhfIpyQ6bsANOY67gAgPxBNXO2TGp7WXAAi+h/JVHjTPBJVMMmah9JURERd1w50biR4+6Mltk44izNVkJuH5RVuxyX2BpK7BlCDsus8/o7280n6CaQeJwqkZwW7WVuzdzyi0JdZL5nmgCHOI65lNQrKKu9ldVA4NBabfk6Lj5kSvd40ue4fUJRzPxPuiSoxgpJ5PFpsmuCPyJN5EOO1EITRSqXvtHZfYChrxIQKjtut+ihbbW8f6y3KeQqpRq5WbQxuQ6cPbuBhJf";

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
    private PositionCalculator positionCalculator;
    private DataLogger dataLogger;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        // initializam voltajul
        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        telemetry.addData("Starting voltage", voltageReader.ReturnVoltage());

        MVTurnTowardsPoint = new MVTurnTowardsPoint();
        telemetry.addData("Status", "Initialized");
        vaccumMotor = hardwareMap.dcMotor.get("VM");
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        push_ring = hardwareMap.servo.get("PR");
        //spagheteMotor = hardwareMap.dcMotor.get("SM");
        right_pistol = hardwareMap.dcMotor.get("RP");
        arm_motor = hardwareMap.dcMotor.get("AM");
        hold_wobble = hardwareMap.servo.get("HW");
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        pistol = new Pistol(right_pistol, voltageReader);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        // aspirator = new Aspirator(vaccumMotor, spagheteMotor);
        holdwobble = new HoldWobble(hold_wobble);
        tragaci = new Tragaci(push_ring);
        aspirator = new Aspirator(vaccumMotor);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        try {
            positionCalculator = new PositionCalculator(move, 0 ,0, rotationDetector, voltageReader);
        } catch (IOException e) {
            e.printStackTrace();
        }
        dataLogger = new DataLogger(rotationDetector, VS, pistol, move, positionCalculator, getClass().getName());



        //webcamDetector = new WebcamDetector(); 
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
            tfod.setZoom(1.75, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");



        waitForStart();





        // verificam daca s-a modificat ceva
        ok=false;
        int number=0;
        if(tfod!=null)
            while (ok==false) {
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
                        label=recognition.getLabel();
                        if(label.equals(LABEL_SECOND_ELEMENT)|| label.equals(LABEL_FIRST_ELEMENT))
                        {
                            ok=true;
                            break;
                        }


                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    //NumberOfRings=i;
                    number++;
                    if(number>=60){
                        ok=true;
                        break;
                    }
                    sleep(50);
                    telemetry.update();

                }


            }
        // am verificat

        if(label!=null)
        {
            if(label.equals(LABEL_SECOND_ELEMENT))
            {
                ZonaB();
                telemetry.addData("Zone", "B");
                telemetry.update();


            }
            if(label.equals(LABEL_FIRST_ELEMENT))
            {
                ZonaC();
                telemetry.addData("Zone","C");
                telemetry.update();

            }
        }
        else if(label==null)
        {

            //ZonaA();
            ZonaA();

            //ZonaA2();

            telemetry.addData("Zone","A");
            telemetry.update();


        }
        else
        {
            telemetry.addData("Zone","NEM");
            telemetry.update();

        }


        telemetry.update();





        // }


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
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    int Unghi;

    private void ZonaANOU()
    {
        /*
        long waitTime = voltageReader.GetWaitTime(180,1);
        Variables.Vector2 newPosition= new Variables.Vector2(0, -180);
        newPosition.TranformToNewPlane(rotationDetector.ReturnPositiveRotation());
        positionCalculator.Position.AddVector(newPosition);
        move.MoveFull(2);
        sleep(waitTime);

         */
        GoTowards(190, 1, -1);
        arm_motor.setPower(-0.4);
        sleep(1000);
        holdwobble.Stop();
        sleep(1000);
        arm_motor.setPower(0);
        arm_motor.setPower(0.4);
        sleep(700);
        arm_motor.setPower(0);
        MVVariables.Vector2 Destination = new MVVariables.Vector2(0, -70);
        GoTo(Destination);
        Unghi=179;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        pistol.Start();
        sleep(3000);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        sleep(300);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(8,3));
        rotate.RotateStop();
        sleep(300);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(300);
        sleep(400);
        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(8,3));
        rotate.RotateStop();
        sleep(300);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(300);
        pistol.Stop();
        sleep(1000);
        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(10,1));
        //GoTo(new Variables.Vector2(-3.72 ,-1.7));
        // -3,72 -1.7
        //
    }

    private void ZonaBNOU()
    {
        /*
        long waitTime = voltageReader.GetWaitTime(180,1);
        Variables.Vector2 newPosition= new Variables.Vector2(0, -180);
        newPosition.TranformToNewPlane(rotationDetector.ReturnPositiveRotation());
        positionCalculator.Position.AddVector(newPosition);
        move.MoveFull(2);
        sleep(waitTime);

         */
        GoTowards(260, 1, -1);
        Unghi=45;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        GoTowards(40,1,-1);
        sleep(200);
        arm_motor.setPower(-0.4);
        sleep(1000);
        holdwobble.Stop();
        sleep(1000);
        arm_motor.setPower(0);
        arm_motor.setPower(0.4);
        sleep(700);
        arm_motor.setPower(0);
        Unghi=0;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(10,1));
        MVVariables.Vector2 Destination = new MVVariables.Vector2(38, -65);
        GoTo(Destination);
        Unghi=179;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        pistol.Start();
        sleep(3000);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        sleep(300);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(8,3));
        rotate.RotateStop();
        sleep(300);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(300);
        sleep(400);
        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(8,3));
        rotate.RotateStop();
        sleep(300);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(300);
        pistol.Stop();
        sleep(1000);
        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(10,1));
        //GoTo(new Variables.Vector2(-3.72 ,-1.7));
        // -3,72 -1.7
        //
    }

    private void ZonaCNOU()
    {
        /*
        long waitTime = voltageReader.GetWaitTime(180,1);
        Variables.Vector2 newPosition= new Variables.Vector2(0, -180);
        newPosition.TranformToNewPlane(rotationDetector.ReturnPositiveRotation());
        positionCalculator.Position.AddVector(newPosition);
        move.MoveFull(2);
        sleep(waitTime);

         */
        GoTowards(350, 1, -1);
        arm_motor.setPower(-0.4);
        sleep(1000);
        holdwobble.Stop();
        sleep(1000);
        arm_motor.setPower(0);
        arm_motor.setPower(0.4);
        sleep(700);
        arm_motor.setPower(0);
        GoTowards(160,1,1);
        MVVariables.Vector2 Destination = new MVVariables.Vector2(0, -70);
        GoTo(Destination);
        sleep(2000);
        Unghi=179;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        pistol.Start();
        sleep(3000);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        sleep(300);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(8,3));
        rotate.RotateStop();
        sleep(300);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(300);
        sleep(400);
        rotate.RotateFull(1);
        sleep(voltageReader.GetWaitTime(8,3));
        rotate.RotateStop();
        sleep(300);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(300);
        pistol.Stop();
        sleep(1000);
        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(10,1));
        //GoTo(new Variables.Vector2(-3.72 ,-1.7));
        // -3,72 -1.7
        //
    }

    // x = distanta
    // i = directia 1 fata, 2 lateral
    // _D = 1 fata, -1 spate
    void GoTowards(float x, int i, int _D){
        long waitTime = voltageReader.GetWaitTime(x,i);
        MVVariables.Vector2 newPosition = new MVVariables.Vector2(0, 0);
        if(i==1) {
            newPosition = new MVVariables.Vector2(x* _D, 0);
            if(_D==-1) {
                move.MoveFull(2);
            }
            else{
                move.MoveFull(1);
            }
        }
        else if(i==2){
            newPosition = new MVVariables.Vector2(0, x* _D);
            if(_D==-1) {
                move.MoveFull(4);
            }
            else{
                move.MoveFull(3);
            }
        }

        newPosition.TranformToNewPlane(rotationDetector.ReturnPositiveRotation());
        positionCalculator.Position.AddVector(newPosition);
        telemetry.addLine("new position: "+ newPosition.SimpleData()+ "\n current position: "+positionCalculator.Position.SimpleData());
        telemetry.update();
        sleep(waitTime);
        move.MoveStop();
    }

    void RotateTo(MVVariables.Vector2 Destinatie){
        int Unghi = (int) MVTurnTowardsPoint.AngleCalculator(Destinatie, positionCalculator.Position, rotationDetector.ReturnPositiveRotation());
        while(rotationDetector.WaitForRotation(Unghi))
        {
            telemetry.addLine("Unghi:"+Unghi);
            TelemetryData();
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop(); // aceasta linie e doar de siguranta.
    }

    void RotateToRelative(MVVariables.Vector2 Destinatie){
        MVVariables.Vector2 Aux = Destinatie;
        Aux.AddVector(positionCalculator.Position);
        int Unghi = (int) MVTurnTowardsPoint.AngleCalculator(Aux, new MVVariables.Vector2(0, 0), rotationDetector.ReturnPositiveRotation());
        while(rotationDetector.WaitForRotation(Unghi))
        {
            telemetry.addLine("Unghi:"+Unghi);
            TelemetryData();
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop(); // aceasta linie e doar de siguranta.
    }

    void GoTo(MVVariables.Vector2 Destinatie){
        MVVariables.Vector2 Distanta = new MVVariables.Vector2(Destinatie.x-positionCalculator.Position.x, Destinatie.y-positionCalculator.Position.y);
        RotateToRelative(Distanta);
        GoTowards((float)Distanta.Magnitude(), 1, 1);

    }
    void TelemetryData(){
        dataLogger.writeData(getRuntime());
        telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));
        telemetry.addLine("Current Rotation: "+ rotationDetector.ReturnPositiveRotation());
        telemetry.update();
    }
    // OLD AUTONOMIE
    private void ZonaA()
    {
        GoTowards(190, 1, -1);
        arm_motor.setPower(-0.4);
        sleep(1000);
        holdwobble.Stop();
        sleep(1000);
        arm_motor.setPower(0);
        arm_motor.setPower(0.4);
        sleep(1300);
        arm_motor.setPower(0);
        Unghi=180;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(12,1));
        move.MoveStop();
        sleep(200);
        pistol.Start();
        sleep(4000);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        sleep(400);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        sleep(400);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(300);
        pistol.Stop();
        sleep(1000);
        Unghi=50;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.RotateStop();
        sleep(200);
        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(40,1));
        move.MoveStop();


    }

    private void ZonaB()
    {
        GoTowards(260, 1, -1);
        Unghi=45;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        GoTowards(15,1,-1);
        sleep(200);
        arm_motor.setPower(-0.4);
        sleep(1200);
        holdwobble.Stop();
        sleep(800);
        arm_motor.setPower(0);
        arm_motor.setPower(0.7);
        sleep(1300);
        arm_motor.setPower(0);
        Unghi=0;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(7,1));
        MVVariables.Vector2 Destination = new MVVariables.Vector2(33, -65);
        GoTo(Destination);
        Unghi=172
        ;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(30,1));
        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(5,1));
        move.MoveStop();
        pistol.Start();
        sleep(3400);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(1000);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(1000);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(300);
        pistol.Stop();
        sleep(1000);
        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(20,1));
    }
    public void ZonaC()
    {
        GoTowards(350, 1, -1);
        arm_motor.setPower(-0.4);
        sleep(1000);
        holdwobble.Stop();
        sleep(1000);
        arm_motor.setPower(0);
        arm_motor.setPower(0.7);
        sleep(700);
        arm_motor.setPower(0);
        GoTowards(165,1,1);
        MVVariables.Vector2 Destination = new MVVariables.Vector2(0, -40);
        GoTo(Destination);
        Unghi=169;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        pistol.Start();
        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(15,1));
        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(3,1));
        move.MoveStop();

        sleep(3400);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        sleep(400);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(400);
        sleep(400);
        tragaci.Start();
        sleep(500);
        tragaci.Stop();
        sleep(300);
        pistol.Stop();
        sleep(1000);
        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(10,1));
    }
    private void Pistol()
    {
        pistol.Start();
        // right_pistol.setPower(1);
    }
}
