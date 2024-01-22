package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOpFinal-RC")
public class telMecBothDrOpt_twoP_RoboCentric extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        boolean driveToggle = true;

//        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
//        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
//Based on testing, the front and back left motors were reversed.  Reversing them here (FIXED THE ISSUE!!!)

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");

        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake_motor");          //pixel intake
        DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter");                //pixel lifter
        lifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo intakeServo = hardwareMap.servo.get("claw");                      //raises the claw arm
        DcMotor robotRaiserMotor = hardwareMap.dcMotor.get("robotRaiser");      //raises the robot using claw

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Field-centric initialization - start
        //**********************************************************************************************************
        //* With the current set up (UsbFacingDirection.LEFT), field centric works, but it assumes that usb ports are pointing at the driver
        //* How might we correct that so that the back of the robot is facing the driver instead???
        //* ...switching the UsbFacingDirection from LEFT to BACKWARD did _not_ have the intended affect
        //*    it shifted the control directions one quarter, so that forward/reverse was a diagonal direction on the joystick
        //* ...switching it to right still oriented the robot so that the usb ports were facing the driver, but it drove awkwardly
        //**********************************************************************************************************

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        //Field-centric initialization - end

        telemetry.setAutoClear(true);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            boolean intakeMotorButtom = gamepad2.b;
            boolean intakeMotorButtomReverse = gamepad2.x;

            float intakeMotorPower = 1f;

            float lifterRaiseMotorPower = gamepad2.right_trigger;
            float lifterLowerMotorPower = gamepad2.left_trigger;

            float raiseRobotPower = 1f;
//            float robotRaiseUpMotorPower = gamepad1.right_trigger;
//            float robotRaiseDownMotorPower = gamepad1.left_trigger;

            if (gamepad1.x) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetry.addData("BL encCurPos: ", backLeftMotor.getCurrentPosition());
            telemetry.addData("LeftJoyX: ", x);
            telemetry.addData("LeftJoyY: ", y);
            telemetry.addData("Heading: ", botHeading);

            //Toggle between field-centric and robot-centric mode
            if (currentGamepad1.x && !previousGamepad1.x) {
                driveToggle = !driveToggle;
            }
            driveToggle = false; //locked in robot centric
            //Perform either type of driving based on the toggle
            if(driveToggle) {
                telemetry.addData("FieldCentric: ", driveToggle);
                //*************************
                //* Field-centric driving *
                //*************************

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

            } else {
                //*************************
                //* Robot-centric driving *
                //*************************
                telemetry.addData("FieldCentric: ", driveToggle);

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                //???: is this something we want to keep for competition (should this be used for both robot and field centric driving (why did we add this?)
                float driveSpeed = 0.65f;

                frontLeftMotor.setPower(frontLeftPower * driveSpeed);
                backLeftMotor.setPower(backLeftPower * driveSpeed);
                frontRightMotor.setPower(frontRightPower * driveSpeed);
                backRightMotor.setPower(backRightPower * driveSpeed);
            }

            //*******************
            //* Extra functions *
            //*******************
            //Pixel Lift
            if (gamepad2.left_trigger > 0) {
                if(Math.abs(lifterMotor.getCurrentPosition()) < 1025) {
                    //pixellifterRaise
                    lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    lifterMotor.setPower(0.25 * (lifterLowerMotorPower));
                } else {
                    lifterMotor.setPower(0f);
                }
                telemetry.addData("Pxlift encCurPos: ", lifterMotor.getCurrentPosition());
            } else if (gamepad2.right_trigger > 0) {
                //pixellifter - Lower
                lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                lifterMotor.setPower(0.3 * (lifterRaiseMotorPower));
                telemetry.addData("Pxlift encCurPos: ", lifterMotor.getCurrentPosition());
            } else {
                //intake
                lifterMotor.setPower(0);
                telemetry.addData("Pxlift encCurPos: ", lifterMotor.getCurrentPosition());
            }

            //Robot Raise
            if (gamepad1.y) {
                //robotRaiser -- up
                robotRaiserMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                robotRaiserMotor.setPower(raiseRobotPower);
            } else if (gamepad1.a) {
                //robotRaiser -- down
                robotRaiserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                robotRaiserMotor.setPower(raiseRobotPower);
            } else {
                robotRaiserMotor.setPower(0F);
            }
/*          if (gamepad1.left_trigger > 0) {
                //robotRaiser -- up
                robotRaiserMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                robotRaiserMotor.setPower(robotRaiseUpMotorPower);
            } else if (gamepad1.right_trigger > 0) {
                //robotRaiser -- down
                robotRaiserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                robotRaiserMotor.setPower(robotRaiseDownMotorPower);
            }
*/

            //Raise the Claw
            telemetry.addData("Claw position: ", intakeServo.getPosition());
            //This is the logic that raises and lowers the claw (game position and hook position)
            if(gamepad1.dpad_up) {
                intakeServo.setPosition(0.25);
            } else if (gamepad1.dpad_down) {
                intakeServo.setPosition(0.75);
            }

            //Spin the intake wheel
            if (intakeMotorButtom) {
                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeMotor.setPower((intakeMotorPower));
                //no extra functions occurring
            } else if (intakeMotorButtomReverse) {
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeMotor.setPower((intakeMotorPower));
                //no extra functions occurring
            } else {
                intakeMotor.setPower(0);
            }
            telemetry.update();
            currentGamepad1.copy(gamepad1);

        }
    }
}

