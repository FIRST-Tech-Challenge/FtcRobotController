package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.libraries.vector.Vector2D;
import org.firstinspires.ftc.teamcode.libraries.MovementCurves.MovementCurves;


@TeleOp
public class RobotFullAbsolute extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        IMU  imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
        //change SLOWSPEED to change how dpad works
        final double SLOWSPEED = .2;

        //change value to change the speed of joysticks
        final double TOTALSPEED = 1;

        double speed;
        double strafe;
        double turn = 0;


        double currentFacing;
        double difference;

        Vector2D direction = new Vector2D(0, 0);
        Vector2D toGo = new Vector2D(0,0);

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
            currentFacing = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            toGo.setVector(-gamepad1.left_stick_y, gamepad1.left_stick_x);
           // toGo.normalizeVector();
             toGo.setRelative(currentFacing);

            speed = toGo.getI() * TOTALSPEED;
            speed = gamepad1.dpad_up ? speed+SLOWSPEED : speed;
            speed = gamepad1.dpad_down ? speed-SLOWSPEED : speed;

            strafe = toGo.getJ() * TOTALSPEED;
            strafe = gamepad1.dpad_right ? strafe+SLOWSPEED : strafe;
            strafe = gamepad1.dpad_left ? strafe-SLOWSPEED : strafe;

            direction.setVector(-gamepad1.right_stick_y, gamepad1.right_stick_x);

            difference = Math.toDegrees(currentFacing-direction.getRadians());


            if (difference > 180) {
                difference -= 360;
            }

            if (difference < -180) {
                difference += 360;
            }

            if (difference > 5 && difference < 180) {
                turn = MovementCurves.roundedSquareCurve(difference/360);
            } else if (difference < -5 && difference > -180) {

                turn = -MovementCurves.roundedSquareCurve(-difference/360);

            } else {
                turn = 0;
            }

            turn *= direction.getMagnitude();

            frontLeftDrive.setPower(speed + strafe + turn);
            frontRightDrive.setPower(speed - strafe - turn);
            backLeftDrive.setPower(speed - strafe + turn);
            backRightDrive.setPower(speed + strafe - turn);


            //the rest of the code goes here

        }
    }
}
