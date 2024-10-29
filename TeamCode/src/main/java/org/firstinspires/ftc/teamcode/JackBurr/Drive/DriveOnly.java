package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Motors.ArmMotorV1;

@TeleOp
public class DriveOnly extends OpMode {
    public RobotV1Config config = new RobotV1Config();

    public DcMotor frontLeft; //PORT 0
    public DcMotor frontRight; //PORT 2
    public DcMotor backLeft; //PORT 1
    public DcMotor backRight; // PORT 3
    public DcMotor slidesMotor; //EXPANSION HUB PORT _

    public double ARM_POWER = config.ARM_POWER;
    public int MOVEMENT_DISTANCE = config.ARM_MOVEMENT_DISTANCE;
    public ElapsedTime armTimer = new ElapsedTime();


    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, config.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, config.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, config.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, config.BACK_RIGHT);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        run_motors();
    }

    public void run_motors(){
        if (config.ARE_MOTORS_UPSIDE_DOWN) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
            double rx = -gamepad1.right_stick_x; //This is reversed for our turning
            drive(y, x, rx);
        }
        else {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            drive(y, x, rx);
        }

    }
    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        telemetry.addData("Front Left: ", frontLeftPower);
        telemetry.addData("Front Right: ", frontRightPower);
        telemetry.addData("Back Left: ", backLeftPower);
        telemetry.addData("Back Right: ", backRightPower);
    }
}

//BLANK CHASSIS:
//0=BL
//1=FL
//2=BR
//3=FR