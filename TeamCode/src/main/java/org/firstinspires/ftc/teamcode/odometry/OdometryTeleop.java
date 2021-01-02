package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drivebase.MecanumDrivebase;


@TeleOp(name = "Robot Teleop Odometry", group = "")
public class OdometryTeleop extends LinearOpMode {

    private MecanumDrivebase mecanumDrivebase = new MecanumDrivebase();

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    //Drive motors
    //DcMotor right_front, right_back, left_front, left_back;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = (8192/5.93687);

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    //String rfName = "FR", rbName = "BR", lfName = "FL", lbName = "BL";
    String verticalLeftEncoderName = "FL", verticalRightEncoderName = "BL", horizontalEncoderName = "BR";

    @Override
    public void runOpMode() throws InterruptedException {

        //float MovementX;
        //float MovementY;
        //float MovementTurn;

        mecanumDrivebase.initialize(this);

        //right_front = hardwareMap.dcMotor.get(rfName);
        //right_back = hardwareMap.dcMotor.get(rbName);
        //left_back = hardwareMap.dcMotor.get(lbName);
        //left_front = hardwareMap.dcMotor.get(lfName);

        //Assign the hardware map to the odometry wheels
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        //Reset the encoders
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */
        //horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_front.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */

        mecanumDrivebase.startControl();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        while(opModeIsActive()){
            // Drive the mecanum chassis
            /* MovementX = gamepad1.left_stick_x;
            MovementY = gamepad1.left_stick_y;
            MovementTurn = gamepad1.right_stick_x;
            right_back.setPower(-MovementY + -MovementTurn + MovementX);
            left_back.setPower(-MovementY + (MovementTurn - MovementX));
            right_front.setPower(-(-MovementY + (-MovementTurn - MovementX)));
            left_front.setPower(-MovementY + MovementTurn + MovementX); */


            mecanumDrivebase.readController(gamepad1);
            mecanumDrivebase.whileOpModeIsActive(this);
            mecanumDrivebase.addTelemetry(telemetry);

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.update();
        }

        //Stop the thread
        mecanumDrivebase.stop();
        globalPositionUpdate.stop();
    }
}