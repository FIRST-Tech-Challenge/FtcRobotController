//package org.firstinspires.ftc.masters.util;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.masters.drive.SampleMecanumDriveCancelable;
//import org.firstinspires.ftc.masters.oldAndUselessStuff.FreightFrenzyTeleOpRed;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@TeleOp(name="Power Play TeleOp", group = "competition")
//public class PowerPlayTeleOp extends LinearOpMode {
//    //RobotClass robot
//    private final FtcDashboard dashboard = FtcDashboard.getInstance();
//
//    private final ElapsedTime runtime = new ElapsedTime();
//
//    DcMotor leftFrontMotor = null;
//    DcMotor rightFrontMotor = null;
//    DcMotor leftRearMotor = null;
//    DcMotor rightRearMotor = null;
//
//    @Override
//    public void runOpMode() {
//
//        telemetry.addData("Status", "Initialized");
//        // telemetry.update();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        //This is the stuff that refers to other stuff that doesn't exist
//        //drive = new SampleMecanumDriveCancelable(hardwareMap, this, telemetry);
//
//        /* Initialize the hardware variables. Note that the strings used here as parameters
//         * to 'get' must correspond to the names assigned during the robot configuration
//         * step (using the FTC Robot Controller app on the phone).
//         */
//        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
//        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
//        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
//        rightRearMotor = hardwareMap.dcMotor.get("backRight");
//
//        // Set the drive motor direction:
//        // "Reverse" the motor that runs backwards when connected directly to the battery
//        // These polarities are for the Neverest 20 motors
//        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
//        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
//        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
//
//        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            switch (currentMode) {
//
//                case NORMAL:
//                    double y = 0;
//                    double x = 0;
//                    double rx = 0;
//                    telemetry.addData("left y", gamepad1.left_stick_y);
//                    telemetry.addData("left x", gamepad1.left_stick_x);
//                    telemetry.addData("right x", gamepad1.right_stick_x);
//
//                    if (Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2) {
//                        y = gamepad1.left_stick_y;
//                        x = gamepad1.left_stick_x;
//                        if (Math.abs(y) < 0.2) {
//                            y = 0;
//                        }
//                        if (Math.abs(x) < 0.2) {
//                            x = 0;
//                        }
//
//                        rx = gamepad1.right_stick_x;
//                    }
//
//                    double leftFrontPower = y + strafeConstant* x + rx;
//                    double leftRearPower = y - strafeConstant* x + rx;
//                    double rightFrontPower = y - strafeConstant* x - rx;
//                    double rightRearPower = y + strafeConstant*x - rx;
//
//
//
//                    if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {
//
//                        double max;
//                        max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
//                        max = Math.max(max, Math.abs(rightFrontPower));
//                        max = Math.max(max, Math.abs(rightRearPower));
//
//                        leftFrontPower /= max;
//                        leftRearPower /= max;
//                        rightFrontPower /= max;
//                        rightRearPower /= max;
//                    }
//
//                    leftFrontMotor.setPower(leftFrontPower * maxPowerConstraint);
//                    leftRearMotor.setPower(leftRearPower * maxPowerConstraint);
//                    rightFrontMotor.setPower(rightFrontPower * maxPowerConstraint);
//                    rightRearMotor.setPower(rightRearPower * maxPowerConstraint);
//
//                    telemetry.addData("left front power", leftFrontPower);
//                    telemetry.addData("left back power", leftRearPower);
//                    telemetry.addData("right front power", rightFrontPower);
//                    telemetry.addData("right back power", rightRearPower);
//
//                    if (gamepad1.a) {
//                        maxPowerConstraint = 1;
//                    }
//
//                    if (gamepad1.x) {
//                        maxPowerConstraint = 0.75;
//                    }
//
//                    if (gamepad1.b) {
//                        maxPowerConstraint = 0.25;
//                    }
//
//
//
//                    if (gamepad1.right_trigger > .35) {
//                        currentMode = FreightFrenzyTeleOpRed.DriveMode.TO_HUB;
//                        boolean foundWhite = toLineTeleop(1.5);
//                        telemetry.addData("found white", foundWhite);
//                        if (foundWhite) {
//
//                            drive.followTrajectoryAsync(toHub);
//                        }
//                    }
//                    break;
////                case TO_HUB:
////
////                    if (!drive.isBusy()) {
////                        currentMode = FreightFrenzyTeleOpRed.DriveMode.NORMAL;
////                    }
////
////                    if (gamepad1.x){
////                        drive.breakFollowing();
////                        currentMode = FreightFrenzyTeleOpRed.DriveMode.NORMAL;
////                    }
////
////                    //add way to stop trajectory
////
////                    break;
////                case TO_SHARED_HUB:
////                    break;
////
////                case TO_WAREHOUSE:
////                    break;
//
//            }
//
//            // start doing stuff here
//
//        }
//
//    }
//
//}
