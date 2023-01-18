package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous (name= "LEFT BABY", group = "Pushbot")
public class redduckspinnew extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private ElapsedTime runtime = new ElapsedTime();
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel",

};
    @Override
    public void runOpMode() {


        HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
        robot.init(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();


        final double FORWARD_SPEED = 0.3;
        final double TURN_SPEED = 0.3;
        int frontRightPosition = 0;
        int frontLeftPosition = 0;
        int backRightPosition = 0;
        int backLeftPosition = 0;

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        //List<Recognition> updatedRecognitions = tfod.getRecognitions();
        waitForStart();


        telemetry.addData("Status", "before reset");
        telemetry.update();
        sleep(200);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "after reset");
        telemetry.update();


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.claw.setPosition(0.3);
        sleep(1000);
        robot.claw.setPosition(-1);
        sleep(1000);

        robot.liftLeft.setPower(0.5);
        robot.liftRight.setPower(0.5);
        sleep(1300);
        robot.liftLeft.setPower(0);
        robot.liftRight.setPower(0);

        frontLeftPosition -= 1100;
        frontRightPosition -= 1100;
        backLeftPosition += 1100;
        backRightPosition += 1100;

        //going backwards

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2500);



        frontLeftPosition -= 2600;
        frontRightPosition += 2600;
        backLeftPosition -= 2600;
        backRightPosition += 2600;
        //going right sideways

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.backLeft.setPower(0.5);
        robot.backRight.setPower(0.5);
        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2500);


        robot.liftLeft.setPower(0.5);
        robot.liftRight.setPower(0.5);
        sleep(3200);
        robot.liftRight.setPower(0.05);
        robot.liftLeft.setPower(0.05);

        frontLeftPosition -= 570;
        frontRightPosition += 570;
        backLeftPosition += 570;
        backRightPosition -= 570;
        //turning towards the pole

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2000);

        frontLeftPosition += 350;
        frontRightPosition += 350;
        backLeftPosition -= 350;
        backRightPosition -= 350;
        //going towards the pole a bit

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setPower(0.3);
        robot.frontRight.setPower(0.3);
        robot.backRight.setPower(0.3);
        robot.backLeft.setPower(0.3);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1300);

        robot.liftLeft.setPower(-0.5);
        robot.liftRight.setPower(-0.5);
        sleep(350);
        robot.liftLeft.setPower(0.05);
        robot.liftRight.setPower(0.05);



        robot.claw.setPosition(1);
        sleep(400);



        frontLeftPosition -= 250;
        frontRightPosition -= 250;
        backRightPosition += 250;
        backLeftPosition += 250;
        //moving back from thw pole

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1000);

        frontLeftPosition += 600;
        frontRightPosition -= 600;
        backLeftPosition -= 600;
        backRightPosition += 600;
        // turning away from the pole

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.liftLeft.setPower(0);
        robot.liftRight.setPower(0);
        sleep(250);
        robot.liftRight.setPower(-0.5);
        robot.liftLeft.setPower(-0.5);
        sleep(250);
        robot.liftRight.setPower(0);
        robot.liftLeft.setPower(0);


    }}




