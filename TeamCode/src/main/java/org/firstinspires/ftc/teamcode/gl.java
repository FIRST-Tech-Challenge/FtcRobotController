package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="pls work",group="Robot")
public class gl extends OpMode{
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    public void init(){
        frontLeftDrive=hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightDrive=hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftDrive=hardwareMap.get(DcMotor.class, "BackLeft");
        backRightDrive=hardwareMap.get(DcMotor.class, "BackRight");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop(){
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backRightPower);
        backRightDrive.setPower(backLeftPower);
    }
}

