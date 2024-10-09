package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="Mecanum Auto Back Right", group="Robot")
public class MecanumAutoBackRight extends LinearOpMode {
    public static final double DRIVE_SPEED = 2.0;
    public static final double TURN_SPEED = 0.5;
    private MecanumRobotController robotController;
    @Override
    public void runOpMode() {
        // Initialize
        initialize();
        waitForStart();
        // robotController.turnTo(45, TURN_SPEED);
        robotController.distanceDrive(29, 180, DRIVE_SPEED);
        robotController.sleep(2);
        robotController.distanceDrive(7, 0, DRIVE_SPEED);
        robotController.turnTo(180, TURN_SPEED);
        robotController.distanceDrive(60, 270, DRIVE_SPEED);
        robotController.distanceDrive(7, 180, DRIVE_SPEED);
        robotController.sleep(2);
        robotController.distanceDrive(24, 0, DRIVE_SPEED);
        robotController.turnTo(135, TURN_SPEED);
        robotController.sleep(5);
    }

    public void initialize() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BACKLEFT");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BACKRIGHT");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FRONTLEFT");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FRONTRIGHT");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        IMU gyro = hardwareMap.get(IMU.class, "imu2");
        gyro.resetYaw();
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, this);
    }
}
