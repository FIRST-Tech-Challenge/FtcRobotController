package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;

import java.util.List;

@Autonomous(name = "Autonomous_Left")
public class Autonomous_Left extends LinearOpMode {
    int detect = 0;
    TurtleRobotAuto robot = new TurtleRobotAuto(this);

    TurtleRobot robot = new TurtleRobot(this);
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   =  3.7795276;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ElapsedTime     runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "AfWHNJL/////AAABmRZFR8RnIEh6pQ3TkKsZ3KBQYvSMKxq99fDcG+7fVkN6hzfwX3fM2jmXufnJdqH+iUwgCcrc0VaCoE/SfyW5426O6eMujAsH4ORkatg1TgmQF4f/BAUPQBDLgDsOnjgWcgJnErKW31ZVbKiGC3gw4mS/6m4kHPOZs3xlViqf3dtNF8pNS7DAlQP5WmzBoTqJF5AMRv6ZOS+kinCoeAsXDme2xNCU7sJWN3TRyLVVpCxt6eJJcx4y4CkLecjdgyGAInok6Pa6AIK9ZnWwr0fg5ABebYd6uujxXn20c0cgH29371C54SvXMrPU2FjmCILQCvDvcnWaseTclX5oauFOJb40RyKq2en4bnFDqShZ14Id";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.0, 16.0/9.0);
        }
        robot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

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
                            if (recognition.getLabel()=="1 Bolt") {detect=1;

                                left(0.25, 500);
                                straight(0.5, 200);
//                                LinearSlide(1, 500);
//                                ServoClaw(0.25, 400);
                                straight(-0.5, 200);
                                right(0.25, 500);
                                straight(0.5, 200);
                                //LinearSlide(1, 500);
                                //ServoClaw(0.25, 400);
                                straight(-0.5, 200);
                                left(0.25, 500);
                                strafeLeft(0.5, 1000);
                                stopRobot();
                                straight(0.5, 1200);
                                stopRobot();
                                stop();
                            }
                            if (recognition.getLabel()=="2 Bulb") {detect=2;
                                left(0.25, 500);
                                straight(0.5, 200);
//                                LinearSlide(1, 500);
//                                ServoClaw(0.25, 400);
                                straight(-0.5, 200);
                                right(0.25, 500);
                                straight(0.5, 200);
                                //LinearSlide(1, 500);
                                //ServoClaw(0.25, 400);
                                straight(-0.5, 200);
                                left(0.25, 500);
                                straight(0.5, 1000);
                                stopRobot();
                                stop();
                            }
                            if (recognition.getLabel()=="3 Panel") {detect=3;
                                left(0.25, 500);
                                straight(0.5, 200);
//                                LinearSlide(1, 500);
//                                ServoClaw(0.25, 400);
                                straight(-0.5, 200);
                                right(0.25, 500);
                                straight(0.5, 200);
                               // LinearSlide(1, 500);
                                //ServoClaw(0.25, 400);
                                straight(-0.5, 200);
                                left(0.25, 500);
                                strafeRight(0.5, 1000);
                                stopRobot();
                                straight(0.5, 1200);
                                stopRobot();
                                stop();
                            }
                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }

                    }
                }
            }

            telemetry.update();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.*/
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

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
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfodParameters.useObjectTracker = false;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    public void straight(double power, int time) {
        robot.leftfrontmotor.setPower(power);
        robot.leftbackmotor.setPower(power);
        robot.rightfrontmotor.setPower(power);
        robot.rightbackmotor.setPower(power);
        sleep(time);
    }
    public void strafeRight(double power, int time) {
        robot.leftfrontmotor.setPower(power);
        robot.leftbackmotor.setPower(-power);
        robot.rightfrontmotor.setPower(-power);
        robot.rightbackmotor.setPower(power);
        sleep(time);
    }
    public void strafeLeft(double power, int time) {
        robot.leftfrontmotor.setPower(-power);
        robot.leftbackmotor.setPower(power);
        robot. rightfrontmotor.setPower(power);
        robot.rightbackmotor.setPower(-power);
        sleep(time);
    }
    public void stopRobot() {
        robot.leftfrontmotor.setPower(0);
        robot.leftbackmotor.setPower(0);
        robot.rightfrontmotor.setPower(0);
        robot.rightbackmotor.setPower(0);
    }
    public void left(double power, int time) {
        robot.leftfrontmotor.setPower(power);
        robot.leftbackmotor.setPower(power);
        robot. rightfrontmotor.setPower(-power);
        robot.rightbackmotor.setPower(-power);
        sleep(time);
    }
    public void right(double power, int time) {
        robot.leftfrontmotor.setPower(-power);
        robot.leftbackmotor.setPower(-power);
        robot. rightfrontmotor.setPower(power);
        robot.rightbackmotor.setPower(power);
        sleep(time);
    }
    public void EncoderDrive(TurtleRobot robot, double speed,
                             double leftfrontInches, double leftbackInches,
                             double rightfrontInches, double rightbackInches,
                             double timeoutS) {
        int newLeftfrontTarget;
        int newLeftbackTarget;
        int newRightfrontTarget;
        int newRightbackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftfrontTarget = robot.leftfrontmotor.getCurrentPosition() + (int) (leftfrontInches * COUNTS_PER_INCH);
            newLeftbackTarget = robot.leftbackmotor.getCurrentPosition() + (int) (leftbackInches * COUNTS_PER_INCH);
            newRightfrontTarget = robot.rightfrontmotor.getCurrentPosition() + (int) (rightfrontInches * COUNTS_PER_INCH);
            newRightbackTarget = robot.rightbackmotor.getCurrentPosition() + (int) (rightbackInches * COUNTS_PER_INCH);
            robot.leftfrontmotor.setTargetPosition(newLeftfrontTarget);
            robot.leftbackmotor.setTargetPosition(newLeftfrontTarget);
            robot.rightfrontmotor.setTargetPosition(newRightfrontTarget);
            robot.rightbackmotor.setTargetPosition(newRightbackTarget);


            // Turn On RUN_TO_POSITION
            robot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftbackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightbackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftfrontmotor.setPower(Math.abs(speed));
            robot.leftbackmotor.setPower(Math.abs(speed));
            robot.rightfrontmotor.setPower(Math.abs(speed));
            robot.rightbackmotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the  will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the  continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (robot.leftfrontmotor.isBusy() &&
                    robot.leftbackmotor.isBusy()
                    && robot.rightfrontmotor.isBusy()
                    && robot.rightbackmotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d",
                        newLeftfrontTarget,
                        newLeftbackTarget,
                        newRightfrontTarget,
                        newRightbackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftfrontmotor.getCurrentPosition(),
                        robot.leftbackmotor.getCurrentPosition(),
                        robot.rightfrontmotor.getCurrentPosition(),
                        robot.rightbackmotor.getCurrentPosition());
                telemetry.update();
            }
            robot.leftfrontmotor.setPower(0);
            robot.leftbackmotor.setPower(0);
            robot.rightfrontmotor.setPower(0);
            robot.rightbackmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
}
