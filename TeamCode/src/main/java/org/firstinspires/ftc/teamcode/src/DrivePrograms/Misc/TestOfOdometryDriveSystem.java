package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.IMU;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.OdometryPodServos;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.OdometryGlobalCoordinatePosition;

@Disabled
@Autonomous(name = "TestOfOdometryDriveSystem")
public class TestOfOdometryDriveSystem extends AutonomousTemplate {

    public void runOpMode() throws InterruptedException {
        podServos = new OdometryPodServos(hardwareMap, "right_odometry_servo", "left_odometry_servo", "horizontal_odometry_servo");
        podServos.lower();


        DcMotor front_right = hardwareMap.dcMotor.get("front_right/vr");
        DcMotor front_left = hardwareMap.dcMotor.get("front_left/vl");
        DcMotor back_left = hardwareMap.dcMotor.get("back_left");
        DcMotor back_right = hardwareMap.dcMotor.get("back_right/h");

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder
        //verticalEncoderLeft, verticalEncoderRight, horizontalEncoder
        odometry = new OdometryGlobalCoordinatePosition(front_left, front_right, back_right, 25, this::opModeIsActive, this::isStopRequested);
        IMU imu = new IMU(hardwareMap, "imu");
        odometry.setImu(imu.getImu());


        odometry.reverseLeftEncoder();
        odometry.start();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Active", odometry.isActive());
            telemetry.addData("Thread State", odometry.isRunning());
            telemetry.addData("Thread State OBJ", odometry.getState());
            odometry.showPosition(telemetry);
            telemetry.update();
        }
    }
}
