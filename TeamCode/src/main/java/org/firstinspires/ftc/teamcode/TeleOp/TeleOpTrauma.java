package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="TeleopTest", group="Trauma")
public class TeleOpTrauma extends LinearOpMode{

    // Declare OpMode members for each of the 4 motors. Yay
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double motorSpeed = 0.85;
//    boolean movingRight;
//    boolean movingLeft;
//    boolean movingForwards;
//    boolean movingBackwards;
    double slowDriveMult = 1;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");



        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if(gamepad1.left_bumper){
                slowDriveMult = 0.5;
            }else{
                slowDriveMult = 1;
            }

            telemetry.addData("leftStick x val:", gamepad1.left_stick_x);
            telemetry.addData("leftStick y val:", gamepad1.left_stick_y);
            telemetry.update();
            //Forward and backward
            if(Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)){
                if (gamepad1.left_stick_y > 0.1) {
                    leftFrontPower = -gamepad1.left_stick_y * motorSpeed * slowDriveMult;
                    rightFrontPower = -gamepad1.left_stick_y * motorSpeed * slowDriveMult;
                    leftBackPower = -gamepad1.left_stick_y * motorSpeed * slowDriveMult;
                    rightBackPower = -gamepad1.left_stick_y * motorSpeed * slowDriveMult;
                } else if (gamepad1.left_stick_y < -0.1) {
                    leftFrontPower = -gamepad1.left_stick_y * motorSpeed * slowDriveMult;
                    rightFrontPower = -gamepad1.left_stick_y * motorSpeed * slowDriveMult;
                    leftBackPower = -gamepad1.left_stick_y * motorSpeed * slowDriveMult;
                    rightBackPower = -gamepad1.left_stick_y * motorSpeed * slowDriveMult;
                } else {
                    leftFrontPower = 0;
                    rightFrontPower = 0;
                    leftBackPower = 0;
                    rightBackPower = 0;
                }
            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            //Strafing
            if(Math.abs(gamepad1.left_stick_x) >= Math.abs(gamepad1.left_stick_y)){
                if (gamepad1.left_stick_x > 0.1){
                    leftFrontPower = gamepad1.left_stick_x*motorSpeed*0.75 * slowDriveMult;
                    rightFrontPower = -gamepad1.left_stick_x*motorSpeed*0.75 * slowDriveMult;
                    leftBackPower = -gamepad1.left_stick_x*motorSpeed*0.75 * slowDriveMult;
                    rightBackPower = gamepad1.left_stick_x*motorSpeed*0.75 * slowDriveMult;
                }else if(gamepad1.left_stick_x < -0.1){
                    leftFrontPower = gamepad1.left_stick_x*motorSpeed*0.75 * slowDriveMult;
                    rightFrontPower = -gamepad1.left_stick_x*motorSpeed*0.75 * slowDriveMult;
                    leftBackPower = -gamepad1.left_stick_x*motorSpeed*0.75 * slowDriveMult;
                    rightBackPower = gamepad1.left_stick_x*motorSpeed*0.75 * slowDriveMult;
                }
            }else{
                leftFrontPower = 0;
                rightFrontPower = 0;
                leftBackPower = 0;
                rightBackPower = 0;
            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if (gamepad1.right_stick_x > 0.1){
                leftFrontPower = -gamepad1.right_stick_x*motorSpeed * slowDriveMult;
                rightFrontPower = gamepad1.right_stick_x*motorSpeed * slowDriveMult;
                leftBackPower = -gamepad1.right_stick_x*motorSpeed * slowDriveMult;
                rightBackPower = gamepad1.right_stick_x*motorSpeed * slowDriveMult;
            }else if(gamepad1.right_stick_x < -0.1){
                leftFrontPower = -gamepad1.right_stick_x*motorSpeed * slowDriveMult;
                rightFrontPower = gamepad1.right_stick_x*motorSpeed * slowDriveMult;
                leftBackPower = -gamepad1.right_stick_x*motorSpeed * slowDriveMult;
                rightBackPower = gamepad1.right_stick_x*motorSpeed * slowDriveMult;
            }else{
                leftFrontPower = 0;
                rightFrontPower = 0;
                leftBackPower = 0;
                rightBackPower = 0;
            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }
    }
}