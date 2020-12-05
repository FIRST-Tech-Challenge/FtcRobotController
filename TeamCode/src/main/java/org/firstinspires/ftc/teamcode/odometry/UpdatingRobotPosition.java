package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

public class UpdatingRobotPosition extends LinearOpMode {

    DcMotor frontRight, frontLeft, backRight, backLeft;
    DcMotor leftEncoder, rightEncoder, middleEncoder;

    static final double TICKS_PER_REV = 8192;
    static final double WHEEL_DIAMETER = 100/25.4;
    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO/TICKS_PER_REV;

    GlobalCoordinateSystem positionUpdate;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        middleEncoder = hardwareMap.dcMotor.get("middleEncoder");

        resetOdometryEncoders();

        waitForStart();

        positionUpdate = new GlobalCoordinateSystem(leftEncoder, rightEncoder, middleEncoder, TICKS_PER_INCH, 100);
        Thread position = new Thread(positionUpdate);
        position.start();

        while (opModeIsActive()) {
            float left = gamepad1.left_stick_y;
            float right = gamepad1.right_stick_x;
            //code for when robot is active

        }
        //divide by ticks per inch so its where it is on the field rather than based on encoder input
        telemetry.addData("X Position", positionUpdate.returnXCoordinate() / TICKS_PER_INCH);
        telemetry.addData("Y Position", positionUpdate.returnYCoordinate() / TICKS_PER_INCH);
        telemetry.addData("orientation(deg)", positionUpdate.returnOrientation());
        telemetry.update();

        positionUpdate.stop();
    }
    private void resetOdometryEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
