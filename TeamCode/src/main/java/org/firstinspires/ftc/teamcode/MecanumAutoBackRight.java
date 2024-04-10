package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="Mecanum Auto Back Right", group="Robot")
public class MecanumAutoBackRight extends LinearOpMode {
    private MecanumRobotController robotController;
    @Override
    public void runOpMode() {
        // Initialize
        initialize();
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
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro);
    }
}
