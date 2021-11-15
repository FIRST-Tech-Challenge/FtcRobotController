package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

@TeleOp(name="teleOperation")
@Disabled
public class teleop extends LinearOpMode {
    MecanumChassis robot;

    @Override
    public void runOpMode() {
        robot = new MecanumChassis();
        robot.init(hardwareMap);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());

        telemetry.addData("IMU Mode", "IMU calibrating....");
        telemetry.update();

        //make sure the IMU gyro is calibrated before continue
        while (!isStopRequested() && !robot.imu.isGyroCalibrated() &&
                !robot.imu.isAccelerometerCalibrated() &&
                !robot.imu.isMagnetometerCalibrated() &&
                !robot.imu.isSystemCalibrated()) {
            idle();
        }

        telemetry.update();
        telemetry.addData("Working Mode", "waiting for start");
        telemetry.update();

        waitForStart();
        //non-refreshing elements


        while (opModeIsActive()) {
            double leftStickXPos = gamepad1.left_stick_x;
            double leftStickYPos = -gamepad1.left_stick_y;
            double rightStickXPos = gamepad1.right_stick_x;


                //fast mode
                leftStickXPos = gamepad1.left_stick_x;
                leftStickYPos = -gamepad1.left_stick_y;
                rightStickXPos = gamepad1.right_stick_x;

            robot.leftFrontDrive.setPower(leftStickYPos + leftStickXPos + rightStickXPos);
            robot.leftRearDrive.setPower(leftStickYPos - leftStickXPos + rightStickXPos);
            robot.rightFrontDrive.setPower(leftStickYPos - leftStickXPos - rightStickXPos);
            robot.rightRearDrive.setPower(leftStickYPos + leftStickXPos - rightStickXPos);

            if(gamepad1.left_bumper){
                robot.intakeUp.setPower(1f);
            }
            else{
                robot.intakeUp.setPower(0f);
            }


            if(gamepad1.right_bumper){
                robot.lift.setPower(0.5f);
            }else if(gamepad1.right_trigger > 0.5f){
                robot.lift.setPower(-0.5f);
            }
            else{
                robot.lift.setPower(0f);
            }


            if(gamepad1.dpad_up){
                robot.arm.setPower(0.5f);
            }
            else if(gamepad1.dpad_down){
                robot.arm.setPower(-0.5f);
            }
            else{
                robot.arm.setPower(0f);
            }

        }

    }
}