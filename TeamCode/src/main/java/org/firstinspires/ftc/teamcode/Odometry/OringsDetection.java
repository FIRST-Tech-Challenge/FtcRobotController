package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My ORings Detection")
public class OringsDetection extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;
    public DcMotor rightShooter;
    public TouchSensor liftButtonBot;
    public CRServo liftservo;
    public TouchSensor liftButtonTop;
    public DcMotor picker;
    public Servo pusher;
    public DcMotor leftShooter;
    public Servo claw;
    public TouchSensor armButtonBot;
    public DcMotor Arm;
    public TouchSensor armButtonTop;


    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "Right_Front_Wheel", rbName = "Right_Rear_Wheel", lfName = "Left_Front_Wheel", lbName = "Left_Rear_Wheel";
    String verticalLeftEncoderName = lbName, verticalRightEncoderName = lfName, horizontalEncoderName = rbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;


    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        // globalPositionUpdate.reverseRightEncoder();
        // globalPositionUpdate.reverseNormalEncoder();

        claw.setPosition(-1);
        while(!liftButtonTop.isPressed()){
            liftservo.setPower(1);
        }

        liftservo.setPower(0);
        GoStraight(
                7 * COUNTS_PER_INCH,
                0.3,
                1 * COUNTS_PER_INCH);

        SlideRight(
                5 * COUNTS_PER_INCH,
                0.3,
                1 * COUNTS_PER_INCH);
        telemetry.addData("The lift servo should start soon", "J");
        telemetry.update();
        Freeze(0);

        telemetry.addData("wE OUT OF THE LOOP", "J");
        telemetry.update();

        //liftservo.setPower(0.1);

        leftShooter.setPower(-1);
        rightShooter.setPower(1);

        sleep(234);
        telemetry.addData("The shooter shooting now", "J");
        telemetry.update();

        pusher.setPosition(1);
        sleep(1000);
        pusher.setPosition(-1);
        sleep(1000);
        SlideRight(
                4 * COUNTS_PER_INCH,
                0.3,
                1 * COUNTS_PER_INCH);
        sleep(1000);

        Freeze(0);


        pusher.setPosition(1);
        sleep(1000);
        pusher.setPosition(-1);
        sleep(1000);
        SlideRight(
                4 * COUNTS_PER_INCH,
                0.3,
                1 * COUNTS_PER_INCH);
        sleep(1000);
        Freeze(0);



        pusher.setPosition(1);
        sleep(1000);
        pusher.setPosition(-1);
        sleep(1000);



        leftShooter.setPower(0);
        rightShooter.setPower(0);
        liftservo.setPower(0);
        telemetry.addData("We done shooting", "J");
        telemetry.update();
        GoStraight(
                60 * COUNTS_PER_INCH,
                0.3,
                1 * COUNTS_PER_INCH);
        SlideLeft(
                20 * COUNTS_PER_INCH,
                0.3,
                1 * COUNTS_PER_INCH);
        Freeze(0);


        do {
           Arm.setPower(0.5);
        }while (!armButtonBot.isPressed());

        Arm.setPower(0);
        claw.setPosition(1);
        sleep(723);

        GoBackward(7 * COUNTS_PER_INCH, 0.3, 1*COUNTS_PER_INCH);
        SlideRight(
                12 * COUNTS_PER_INCH,
                0.3,
                1 * COUNTS_PER_INCH);
        GoStraight(
                7 * COUNTS_PER_INCH,
                0.3,
                1 * COUNTS_PER_INCH);
        /*
        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", globalPositionUpdate.verticalLeftEncoderWheelPosition);
            telemetry.addData("Vertical right encoder position", globalPositionUpdate.verticalRightEncoderWheelPosition);
            telemetry.addData("Horizontal encoder position", globalPositionUpdate.normalEncoderWheelPosition);

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }
*/
        //Stop the thread
        globalPositionUpdate.stop();
    }

    public void GoStraight(double targetPosition, double robotPower, double allowableDistanceError){

        double distanceToTarget = targetPosition + globalPositionUpdate.verticalRightEncoderWheelPosition;

        while(opModeIsActive() && distanceToTarget > allowableDistanceError) {

            distanceToTarget = targetPosition - globalPositionUpdate.verticalRightEncoderWheelPosition;

            right_back.setPower(robotPower);
            right_front.setPower(robotPower);
            left_front.setPower(robotPower);
            left_back.setPower(robotPower);

            telemetry.addData("YCoordinate", globalPositionUpdate.verticalRightEncoderWheelPosition);
            telemetry.update();
        }
    }

    public void SlideLeft(double targetPosition, double robotPower, double allowableDistanceError){

        double distanceToTarget = targetPosition - globalPositionUpdate.GetHorizontalEncoderPosition();

        while(opModeIsActive() && distanceToTarget > allowableDistanceError) {

            distanceToTarget = targetPosition +
                    globalPositionUpdate.GetHorizontalEncoderPosition();

            right_back.setPower(robotPower * -1);
            right_front.setPower(robotPower);
            left_front.setPower(robotPower * -1);
            left_back.setPower(robotPower);

            telemetry.addData("XCoordinate", globalPositionUpdate.GetHorizontalEncoderPosition());

            telemetry.update();
        }
    }

    public void SlideRight(double targetPosition, double robotPower, double allowableDistanceError){

        double distanceToTarget = targetPosition + globalPositionUpdate.GetHorizontalEncoderPosition();

        while(opModeIsActive() && distanceToTarget > allowableDistanceError) {

            distanceToTarget = targetPosition - globalPositionUpdate.GetHorizontalEncoderPosition();

            right_back.setPower(robotPower);
            right_front.setPower(robotPower * -1);
            left_front.setPower(robotPower);
            left_back.setPower(robotPower * -1);

            telemetry.addData("XCoordinate", globalPositionUpdate.GetHorizontalEncoderPosition());
            telemetry.update();
        }
    }

    public void GoBackward(double targetPosition, double robotPower, double allowableDistanceError){

        double distanceToTarget = targetPosition - globalPositionUpdate.verticalRightEncoderWheelPosition;

        while(opModeIsActive() && distanceToTarget > allowableDistanceError) {

            distanceToTarget = targetPosition + globalPositionUpdate.verticalRightEncoderWheelPosition;

            right_back.setPower(robotPower * -1);
            right_front.setPower(robotPower * -1);
            left_front.setPower(robotPower * -1);
            left_back.setPower(robotPower * -1);

            telemetry.addData("YCoordinate", globalPositionUpdate.verticalRightEncoderWheelPosition);
            telemetry.update();
        }
    }
    public void Freeze(double robotPower){


            right_back.setPower(0);
            right_front.setPower(0);
            left_front.setPower(0);
            left_back.setPower(0);


        }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        liftButtonBot = hardwareMap.get(TouchSensor.class, "liftButtonBot");
        liftservo = hardwareMap.get(CRServo.class, "liftservo");
        liftButtonTop = hardwareMap.get(TouchSensor.class, "liftButtonTop");
        picker = hardwareMap.get(DcMotor.class, "picker");
        pusher = hardwareMap.get(Servo.class, "pusher");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");

        claw = hardwareMap.get(Servo.class, "claw");
        armButtonBot = hardwareMap.get(TouchSensor.class, "armButtonBot");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        armButtonTop = hardwareMap.get(TouchSensor.class, "armButtonTop");


        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}