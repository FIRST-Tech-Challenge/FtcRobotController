package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.libraries.vector.Vector2D;
import org.firstinspires.ftc.teamcode.libraries.MovementCurves.MovementCurves;


@TeleOp
public class RobotAbsoluteTurn extends LinearOpMode {

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
        double angleDifference;

        Vector2D direction = new Vector2D(0, 0);

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
            currentFacing = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            speed = -gamepad1.left_stick_y * TOTALSPEED;
            speed = gamepad1.dpad_up ? speed+SLOWSPEED : speed;
            speed = gamepad1.dpad_down ? speed-SLOWSPEED : speed;

            strafe = gamepad1.left_stick_x * TOTALSPEED;
            strafe = gamepad1.dpad_right ? strafe+SLOWSPEED : strafe;
            strafe = gamepad1.dpad_left ? strafe-SLOWSPEED : strafe;

            direction.setVector(gamepad1.right_stick_x, -gamepad1.right_stick_y);

            angleDifference = currentFacing-direction.getAngle();


            if (angleDifference > 180) {
                angleDifference -= 360;
            }

            if (angleDifference < -180) {
                angleDifference += 360;
            }

            if (angleDifference > 2 && angleDifference < 180) {
                turn = MovementCurves.circleCurve(angleDifference/360);
            } else if (angleDifference < -2 && angleDifference > -180) {

                turn = -MovementCurves.circleCurve(-angleDifference/360);

            } else {
                turn = 0;
            }




            turn *= direction.getMagnitude();



            frontLeftDrive.setPower(speed + strafe + turn);
            frontRightDrive.setPower(speed - strafe - turn);
            backLeftDrive.setPower(speed - strafe + turn);
            backRightDrive.setPower(speed + strafe - turn);


            telemetry.addData("rightStickx", gamepad1.right_stick_x);
            telemetry.addData("leftSticky", gamepad1.right_stick_y);
            telemetry.addData("currentFacing", currentFacing);
            telemetry.addData("wanted angle", direction.getAngle());
            telemetry.addData("angleDifference", angleDifference);
            telemetry.addData("vec Mag", direction.getMagnitude());
            telemetry.update();

        }
    }
}
