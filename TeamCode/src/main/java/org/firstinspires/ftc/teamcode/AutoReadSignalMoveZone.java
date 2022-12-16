package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoReadSignalMoveZone", group = "Linear Opmode")
//@Disabled

public class AutoReadSignalMoveZone extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is i--ncluded in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/team3666.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    //drivetrain
    public DcMotorEx leftDrive;
    public DcMotorEx rightDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightBackDrive;



    public ElapsedTime runtime = new ElapsedTime();

    public double off = 0.0;
    double corrections = 1.0;
    double maxVelocity = 340 * 537.6; //for AndyMark NeveRest 20 motor
    boolean hasMoved = false;
    int count = 0;


    private static final String VUFORIA_KEY =
            "AclDUAH/////AAABmYzSWAdyDktyn7LeKaYpXPkeHMDuWfVt+ZWKtbsATYUHu+lKEe6ywQGLZLm5MRmxfQ4UQRSZ8hR7Hx7cwiYcj7DBcqr2CcI/KXvXFnaoaSHonQcH5UjgGwygyR0DRMvRI9Mm+MnWqdwgQuS4eNYgz/vAuNpeGRJmwimGZkb9kb9Uai+RaH2V33PvH4TZepOg//RReZrL33oLxaLEchTHATEKR1xj6NLzHuZVuOTnIaMwPHRrkkK/cyMqaog/be+k2uxxQ2Lxtb2Yb4nHt4n8Rs7ajT/dUSsP/6pZdWmVs7BmIafbHlLFlS/6+1rDbSfOHqEyHFoLDq/hselgdVG2pzEzPcr3ntMwoIAPjiA799i5";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        setUpHardware();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                        telemetry.update();
                        if(updatedRecognitions.size()>0){
                            if(!hasMoved){
                                moveToZone(updatedRecognitions.get(0).getLabel(),count);
                            }
                        }else{
                            setVelo(true,.1,5);
                            count++;
                        }

                        //instead of above if else statement:
                        /*
                        while(true){
                            if(updatedRecognitions.size() > 0){
                                if(!hasMoved){
                                    //change move to zone to account for positioning
                                    moveToZone(updatedRecognitions.get(0).getLabel(),count);
                                }
                            }else{
                                strafeVelo(true,.25,.5);
                                count++;
                            }
                        }
                        */
                    }
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */

    /* Change this function completely:
     * Use this with strafe function paired with the count variable found in runOpMode. For example,
     * to get to location 1 (left side of the arena), the robot needs to move about one foot from
     * starting position to the left, and forward either one or two feet. Since the robot strafes a
     * little to the left before reading the image on the cone, the code needs to correct the distance
     * that needs to be traveled using the count function. To do this, first you need to figure out the
     * relationship between power and time for the motors. For example, if the robot moves one foot with
     * a power of .05 and time of 5 seconds, and also moves one foot with a power of .25 and a time of 1
     * seconds, then the equation for both would be power * x and time / x, assuming x is the same for both
     * equations. If this was the equation, a potential code for moving the rest of the one foot to the
     * left, assuming power used in count is .05 and the time was .5, the power used in the function is
     * -.25 and the time is 4.5, and the number stored in count is 3, could be: power = -.25,
     * time = 4.5 - (.5 / count). Until the conversion rate is figured out, the current code works
     * perfectly fine for what it needs to do (kind of).
     *
     * In theory, using DcMotorEx, you can use getVelocity() to calculate distance travelled by multiplying
     * it by time, and then applying that to distance from starting pos. Maybe set a few variables for
     * times triggered, and the velocity over that time? or maybe just use .setVelocity() instead of
     * .setPower() that was it's more reliable (v*t = d)
     */
    public void moveToZone(String s, int count){
        telemetry.addData("Detected:", s);

        if(s.equals("1 Bolt")){
            F(.25,4.5);
            fortyFive(-.5,1.5);//90
            F(.25,2);
            hasMoved = true;
        }


        if(s.equals("2 Bulb")){
            F(.25,4.5);
            hasMoved = true;
        }

        if(s.equals("3 Panel")){
            F(.25,4.5);
            fortyFive(.5,1.5);//90
            F(.25,2);
            hasMoved = true;
            /*strafe(false,.25,2);
            F(.25,4.5);*/
        }
    }


    public void setUpHardware() {
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        /* Maybe use this to fix needing to use negatives in the functions below
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        */
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        // tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }


    //Driving functions. Using negative power since rn negative is forward and it's easier to understand when you don't need to add it in when calling the function
    public void F(double power, double time){
        leftRight(-power,-power,time);
    }
    public void B(double power, double time){
        leftRight(power,power,time);
    }
    public void fortyFive(double power,double time){
        runtime.reset();
        while(runtime.seconds() < time) {
            LW(-power);
            RW(-1*-power);
        }
        if(runtime.seconds() >= time){
            LW(off);
            RW(off);
        }
    }

    public void leftRight(double powerLeft, double powerRight){
        LW(powerLeft);
        LBW(powerLeft);
        RW(powerRight);
        RBW(powerRight);
    }

    //overloaded leftRight
    public void leftRight(double powerLeft, double powerRight, double time){
        runtime.reset();
        while(runtime.seconds() < time) {
            leftRight(powerLeft,powerRight);
        }
        if(runtime.seconds() >= time){
            motorsOff();
        }
    }

    public void strafe(boolean isLeft, double power, double time){
        runtime.reset();
        int change = -1;
        if(isLeft){
            change *= -1;
        }
        while(runtime.seconds() < time) {
            RW( -power *-1 *change);
            LBW(-power *-1 * change);
            RBW( -power *change);
            LW( -power *change);
        }
        if(runtime.seconds() >= time){
            motorsOff();
        }
    }


    //Drive functions using .setVelocity
    public void setVelo(boolean forward, double maxPercent, double time){
        //Forward or backward
        int direction = -1;
        if(forward){
            direction = 1;
        }
        runtime.reset();
        while(runtime.seconds() < time){
            leftVelo(maxPercent*direction);
            rightVelo(maxPercent*direction);
        }
        if(runtime.seconds() >= time){
            motorsOff();
        }
    }

    public void strafeVelo(boolean isLeft, double maxPercent, double time){
        //Strafe left or right
        int direction = -1;
        if(isLeft){
            direction = 1;
        }
        runtime.reset();
        while(runtime.seconds() < time){
            leftWheel(maxPercent * direction);
            rightWheel(maxPercent * direction * -1);
            leftBackWheel(maxPercent * direction * -1);
            rightBackWheel(maxPercent * direction);
        }
        if(runtime.seconds() >= time){
            motorsOff();
        }
    }

    public void turnNinety(boolean CW){
        //Turns Ninety degrees
        //currently not measured, will have to test this to be exact
        if(CW){
            //left = +; right = -
            leftVelo(.75);
            rightVelo(-.75);
        }else{
            //left = -; right = +
            leftVelo(-.75);
            rightVelo(.75);
        }
    }

    public void leftVelo(double maxPercent){ //sets power for left wheels
        leftWheel(maxPercent);
        leftBackWheel(maxPercent);
    }

    public void rightVelo(double maxPercent){ //sets power for right wheels
        rightWheel(maxPercent);
        rightBackWheel(maxPercent);
    }


    //Primitive functions

    public void waitTime(double time){
        runtime.reset();
        while(runtime.seconds()<time){
        }
    }

    public void motorsOff(){ //turns all motors off
        leftDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    //Power each motor individually
    public void LW(double d) {
        leftDrive.setPower(d*corrections*-1);
    }
    public void RW(double d) {
        rightDrive.setPower(d * corrections);
    }
    public void LBW(double d) {
        leftBackDrive.setPower(d*corrections*-1);
    }
    public void RBW(double d) {
        rightBackDrive.setPower(d * corrections);
    }


    //Power each motor with .setVelocity()
    public void leftWheel(double percent){
        leftDrive.setVelocity(-maxVelocity*percent);
    }
    public void rightWheel(double percent){
        rightDrive.setVelocity(maxVelocity*percent);
    }
    public void leftBackWheel(double percent){
        leftBackDrive.setVelocity(-maxVelocity*percent);
    }
    public void rightBackWheel(double percent){
        rightBackDrive.setVelocity(maxVelocity*percent);
    }
}