package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config

public class argonautRobot extends LinearOpMode {

//PID-start
    public static int pxlLiftUp, pxlLiftDown, pxlLiftIntake;

    public static double reducePower;
    public BasicPIDController controller;
    private DcMotorEx lift;
    public static int targetPosDown, targetPosHover, targetPosUp = 0;

    public int targetPosition = 0;
    public static double p = 0.00, i = 0, d = 0;
//PID-end

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        //Define variables to robot components
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake_motor");          //pixel intake
        DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter");                //pixel lifter

        Servo clawServo = hardwareMap.servo.get("claw");                      //raises the claw arm
        Servo droneServo = hardwareMap.servo.get("drone");                      //raises the claw arm
        DcMotor robotRaiserMotor = hardwareMap.dcMotor.get("robotRaiser");      //raises the robot using claw

        //Initial motor configurations
        lifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //PID-start
        controller = new BasicPIDController(p, i, d);
        lift = hardwareMap.get(DcMotorEx.class, "lifter");
        lift.setDirection(DcMotorEx.Direction.REVERSE);
        //PID-stop

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
        imu.resetYaw();
        telemetry.setAutoClear(true);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Define variables to controller actions
            //Player 1
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            boolean robotRaising = gamepad1.y;
            boolean robotLowering = gamepad1.a;
            boolean clawArmToUp = gamepad1.dpad_up;
            boolean clawArmToDown = gamepad1.dpad_down;

            //Player 2
            boolean intakeMotorButton = gamepad2.b;
            boolean intakeMotorButtonReverse = gamepad2.x;

//            boolean pidPixelArmHigh = gamepad2.dpad_up;         //deliver position
//            boolean pidPixelArmMedium = gamepad2.dpad_left;     //hover position
//            boolean pidPixelArmLow = gamepad2.dpad_down;        //intake position

            float lifterLowerMotorPower = gamepad2.right_trigger;
            float lifterRaiseMotorPower = gamepad2.left_trigger;

            boolean droneToSetPosition = gamepad2.dpad_left;
            boolean droneToLaunch = gamepad2.dpad_right;
            boolean droneFailSafe = gamepad2.right_bumper;

            //Configurations
            float intakeMotorPower = 1f;
            float raiseRobotPower = 1f;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double secondLiftDesignBoost = 1.15;     //for original design it is 1
            //original
            int LOWER_PIXEL_ARM_LIMIT = 40;
            int MIDDLE_PIXEL_ARM_TARGET = 200;
            int UPPER_PIXEL_ARM_LIMIT = 4000;
            //integrated intake


            //double botHeading2 = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


            telemetry.addData("BL encCurPos: ", backLeftMotor.getCurrentPosition());
            telemetry.addData("LeftJoyX: ", x);
            telemetry.addData("LeftJoyY: ", y);
            telemetry.addData("Heading: ", botHeading);

            drive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, x, y, rx, botHeading);

            dashboardTelemetry.addData("currentPose", lift.getCurrentPosition());

            //*******************
            //* Extra functions *
            //*******************
            //PID-controlled - pixel lift - start
/*
            if(pidPixelArmHigh) {
                //PID-start
                targetPosition = targetPosUp;
//                controller.setP(p);
//                double liftPower = controller.calculate(Math.abs(lift.getCurrentPosition()), targetPosUp);
//                lift.setPower(liftPower);
                telemetry.addData("currentPose", lift.getCurrentPosition());
                dashboardTelemetry.addData("targetPosition", targetPosition);
                dashboardTelemetry.addData("currentPose", lift.getCurrentPosition());
                dashboardTelemetry.addData("power", lift.getPower());
//                telemetry.update();
                //PID-end
            } else if(pidPixelArmMedium) {
                    //PID-start
                targetPosition = targetPosHover;
//                    controller.setP(p);
//                    double liftPower = controller.calculate(Math.abs(lift.getCurrentPosition()), targetPosHover);
//                    lift.setPower(liftPower);
                    telemetry.addData("currentPose", lift.getCurrentPosition());
                    dashboardTelemetry.addData("targetPosition", targetPosition);
                    dashboardTelemetry.addData("currentPose", lift.getCurrentPosition());
                    dashboardTelemetry.addData("power", lift.getPower());
//                    telemetry.update();
                    //PID-end
            } else if(pidPixelArmLow) {
                //PID-start
                targetPosition = targetPosDown;
//                controller.setP(p);
//                double liftPower = controller.calculate(Math.abs(lift.getCurrentPosition()), targetPosDown);
//                lift.setPower(liftPower);
                telemetry.addData("currentPose", lift.getCurrentPosition());
                dashboardTelemetry.addData("targetPosition", targetPosition);
                dashboardTelemetry.addData("currentPose", lift.getCurrentPosition());
                dashboardTelemetry.addData("power", lift.getPower());
//                dashboardTelemetry.update();
//                telemetry.update();

                //PID-end
            }

            //PID-start
            controller.setP(p);
            double liftPower = reducePower * controller.calculate(Math.abs(lift.getCurrentPosition()), targetPosition);
            lift.setPower(liftPower);
            dashboardTelemetry.addData("targetPosition", targetPosition);
            //PID-end

            dashboardTelemetry.addData("power", lift.getPower());
            ////PID-controlled - pixel lift - end
*/

//...playing with code to see if we can reset in encoders...
//            lifterMotor.resetDeviceConfigurationForOpMode();
//            lifterMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
/*
            //Pixel Lift - manual control - start
            if (lifterRaiseMotorPower > 0) {

                if(Math.abs(lifterMotor.getCurrentPosition()) < 695 &&
                   Math.abs(lifterMotor.getCurrentPosition()) > 600) {                 //was 1025 with the old motor, now 701 (using a value that is 1% less than that)


                    //pixellifterRaise
                    lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.3 * (lifterRaiseMotorPower));

                } else if(Math.abs(lifterMotor.getCurrentPosition()) < 600) {                 //was 1025 with the old motor, now 701 (using a value that is 1% less than that)
                    //pixellifterRaise
                    lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.5 * (lifterRaiseMotorPower));

                } else {
                    lifterMotor.setPower(0f);
                }
            } else if (lifterLowerMotorPower > 0) {

                if(Math.abs(lifterMotor.getCurrentPosition()) > 600) {
                    //pixellifter - Lower
                    lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.7 * (lifterLowerMotorPower));

                } else if(Math.abs(lifterMotor.getCurrentPosition()) <= 600 &&
                          Math.abs(lifterMotor.getCurrentPosition()) > 200) {
                    //pixellifter - Lower
                    lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.5 * (lifterLowerMotorPower));

                } else if(Math.abs(lifterMotor.getCurrentPosition()) > 40) {
                    //pixellifter - Lower
                    lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.3 * (lifterLowerMotorPower));

                } else {
                    lifterMotor.setPower(0f);
                }
            } else {
                //intake
                lifterMotor.setPower(0);
            }
*/

            if (lifterRaiseMotorPower > 0) {

                if(Math.abs(lifterMotor.getCurrentPosition()) < (UPPER_PIXEL_ARM_LIMIT + 95) &&
                        Math.abs(lifterMotor.getCurrentPosition()) > UPPER_PIXEL_ARM_LIMIT) {                 //was 1025 with the old motor, now 701 (using a value that is 1% less than that)


                    //pixellifterRaise
                    lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.3 * (lifterRaiseMotorPower));

                } else if(Math.abs(lifterMotor.getCurrentPosition()) < UPPER_PIXEL_ARM_LIMIT) {                 //was 1025 with the old motor, now 701 (using a value that is 1% less than that)
                    //pixellifterRaise
                    lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.5 * (lifterRaiseMotorPower));

                } else {
                    lifterMotor.setPower(0f);
                }
            } else if (lifterLowerMotorPower > 0) {

                if(Math.abs(lifterMotor.getCurrentPosition()) > UPPER_PIXEL_ARM_LIMIT) {
                    //pixellifter - Lower
                    lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.7 * (lifterLowerMotorPower));

                } else if(Math.abs(lifterMotor.getCurrentPosition()) <= UPPER_PIXEL_ARM_LIMIT &&
                        Math.abs(lifterMotor.getCurrentPosition()) > MIDDLE_PIXEL_ARM_TARGET) {
                    //pixellifter - Lower
                    lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.5 * (lifterLowerMotorPower));

                } else if(Math.abs(lifterMotor.getCurrentPosition()) > LOWER_PIXEL_ARM_LIMIT) {
                    //pixellifter - Lower
                    lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    lifterMotor.setPower(secondLiftDesignBoost * 0.3 * (lifterLowerMotorPower));

                } else {
                    lifterMotor.setPower(0f);
                }
            } else {
                //intake
                lifterMotor.setPower(0);
            }

            telemetry.addData("Pxlift encCurPos: ", lifterMotor.getCurrentPosition());
            dashboardTelemetry.addData("currentPose", lift.getCurrentPosition());
            dashboardTelemetry.addData("power", lift.getPower());

            //Pixel Lift - manual control - end

            //Robot Raise
            if (robotRaising) {
                //robotRaiser -- up
                robotRaiserMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                robotRaiserMotor.setPower(raiseRobotPower);
            } else if (robotLowering) {
                //robotRaiser -- down
                robotRaiserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                robotRaiserMotor.setPower(raiseRobotPower);
            } else {
                robotRaiserMotor.setPower(0F);
            }

            //Raise the Claw
            telemetry.addData("Claw position: ", clawServo.getPosition());
            //This is the logic that raises and lowers the claw (game position and hook position)
            if(clawArmToUp) {
                clawServo.setPosition(0.25);
            } else if (clawArmToDown) {
                clawServo.setPosition(0.75);
            }

            //Launch the drone
            telemetry.addData("Drone position: ", droneServo.getPosition());
            //This is the logic that raises and lowers the claw (game position and hook position)
            if(droneFailSafe && droneToSetPosition) {
                droneServo.setPosition(0.12);       //55
            }
            if (droneFailSafe && droneToLaunch) {
                droneServo.setPosition(0.55);       //90
            }

            //Spin the intake wheel
            if (intakeMotorButton) {
//                //pixellifter - Lower
//                if(Math.abs(lifterMotor.getCurrentPosition()) > 0) {
//                    lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//                    lifterMotor.setPower(0.5);
//                }
                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeMotor.setPower((intakeMotorPower));
                //no extra functions occurring
            } else if (intakeMotorButtonReverse) {
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeMotor.setPower((intakeMotorPower));
                //no extra functions occurring
            } else {
                intakeMotor.setPower(0);
            }
            dashboardTelemetry.update();
            telemetry.update();
        }
    }

    //This method is overridden by the child with the appropriate type of drive
    public void drive(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor,
                               double x, double y, double rx, double botHeading) {}
}

