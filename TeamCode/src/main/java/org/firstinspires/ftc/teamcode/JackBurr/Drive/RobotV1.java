package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RobotV1 extends OpMode {
    public RobotV1Config config = new RobotV1Config();

    public DcMotor frontLeft; //PORT 3
    public DcMotor frontRight; //PORT 1
    public DcMotor backLeft; //PORT 2
    public DcMotor backRight; // PORT 4

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, config.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, config.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, config.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, config.BACK_RIGHT);
    }

    @Override
    public void loop() {

    }

    public void drive(){
        MecanumDrive drive = new MecanumDrive();
        if(config.ARE_MOTORS_UPSIDE_DOWN) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
            double rx = -gamepad1.right_stick_x; //This is reversed for our turning
            drive.drive(y, x, rx);
        }
        else {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            drive.drive(y, x, rx);
        }

    }
}
