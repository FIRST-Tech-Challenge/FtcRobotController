package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp
public class RobotFullRelative extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        //change SLOWSPEED to change how dpad works
        final double SLOWSPEED = .2;

        //change value to change the speed of joysticks
        final double TOTALSPEED = 1;

        double speed;
        double strafe;
        double turn;

        DcMotor backRightDrive = null;
        DcMotor frontRightDrive = null;
        DcMotor frontLeftDrive = null;
        DcMotor backLeftDrive = null;

        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);



        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            speed = -gamepad1.left_stick_y * TOTALSPEED;
            speed = gamepad1.dpad_up ? speed+SLOWSPEED : speed;
            speed = gamepad1.dpad_down ? speed-SLOWSPEED : speed;

            strafe = gamepad1.left_stick_x * TOTALSPEED;
            strafe = gamepad1.dpad_right ? strafe+SLOWSPEED : strafe;
            strafe = gamepad1.dpad_left ? strafe-SLOWSPEED : strafe;

            turn = gamepad1.right_stick_x * TOTALSPEED;

            frontLeftDrive.setPower(speed + strafe + turn);
            frontRightDrive.setPower(speed - strafe - turn);
            backLeftDrive.setPower(speed - strafe + turn);
            backRightDrive.setPower(speed + strafe - turn);
        }
    }
}