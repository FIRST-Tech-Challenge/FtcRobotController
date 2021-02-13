package org.firstinspires.ftc.teamcode.components.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//import org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */
@Disabled
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotorEx right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotorEx verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "frontRight" , lfName = "frontLeft";
    String rbName;
    String lbName;
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75, 2);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.tryGet(DcMotorEx.class,rfName);
        right_back = hardwareMap.tryGet(DcMotorEx.class,rbName);
        left_front = hardwareMap.tryGet(DcMotorEx.class,lfName);
        left_back = hardwareMap.tryGet(DcMotorEx.class,lbName);

        verticalLeft = hardwareMap.tryGet(DcMotorEx.class,vlEncoderName);
        verticalRight = hardwareMap.tryGet(DcMotorEx.class,vrEncoderName);
        horizontal = hardwareMap.tryGet(DcMotorEx.class,hEncoderName);

        right_front.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorEx.Direction.REVERSE);
        right_front.setDirection(DcMotorEx.Direction.REVERSE);
        right_back.setDirection(DcMotorEx.Direction.REVERSE);

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
