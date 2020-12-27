package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OdometryControls {

    double _xCoordinate;
    double _yCoordinate;
    double _orientation;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = (8192/5.93687);

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    //String rfName = "FR", rbName = "BR", lfName = "FL", lbName = "BL";
    String verticalLeftEncoderName = "LftOdometry", verticalRightEncoderName = "Intake", horizontalEncoderName = "CntrOdometry";

    //OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);

    public void initialize(LinearOpMode op) {

        //Assign the hardware map to the odometry wheels
        verticalLeft = op.hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = op.hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = op.hardwareMap.dcMotor.get(horizontalEncoderName);

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

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startControl() {
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
    }

    public void whileOpModeIsActive (LinearOpMode op) {

    }

    public void returnCoordinates () {
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        _xCoordinate = globalPositionUpdate.returnXCoordinate();
        _yCoordinate = globalPositionUpdate.returnYCoordinate();
        _orientation = globalPositionUpdate.returnOrientation();
    }

    public void addTelemetry (Telemetry telemetry) {
        returnCoordinates();
        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", _xCoordinate / COUNTS_PER_INCH);
        telemetry.addData("Y Position", _yCoordinate / COUNTS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", _orientation);
        telemetry.update();
    }

    public void stop () {
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        globalPositionUpdate.stop();
    }
}