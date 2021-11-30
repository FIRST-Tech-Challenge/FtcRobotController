package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "yxortele")
public class FreightFrenzy extends LinearOpMode {
    MecanumChassis robot = new MecanumChassis();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

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

        boolean fast = true;
        double modifier = 1;

        while(opModeIsActive()){

            telemetry.update();

            double fl;
            double fr;
            double bl;
            double br;

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if(gamepad1.a){
                fast = !fast;
            }

            if(fast){
                modifier = 1;
            } else {
                modifier = 0.5;
            }

            fl = Range.clip((-drive-strafe-turn), -0.7, 0.7);
            fr = Range.clip((-drive+strafe+turn), -0.7, 0.7);
            bl = Range.clip((-drive+strafe-turn), -0.7, 0.7);
            br = Range.clip((-drive-strafe+turn), -0.7, 0.7);

            if(gamepad1.right_trigger > 0.5){
                robot.intakeUp.setPower(1);
            } else if(gamepad1.right_bumper){
                robot.intakeUp.setPower(-1);
            } else{
                robot.intakeUp.setPower(0);
            }

            if(gamepad1.left_trigger > 0.5){
                robot.lift.setPower(0.7);
            } else if(gamepad1.left_bumper){
                robot.lift.setPower(-0.7);
            } else{
                robot.lift.setPower(0);
            }

            if(gamepad1.dpad_up){
                robot.exten.setPower(0.7);
            } else if(gamepad1.dpad_down){
                robot.exten.setPower(-0.7);
            } else{
                robot.exten.setPower(0);
            }

            if(gamepad1.b){
                robot.duck.setPower(0.3);
            } else{
                robot.duck.setPower(0);
            }


            robot.leftFrontDrive.setPower(modifier*fl);
            robot.rightFrontDrive.setPower(modifier*fr);
            robot.leftRearDrive.setPower(modifier*bl);
            robot.rightRearDrive.setPower(modifier*br);

            telemetry.addData("Front Left", robot.leftFrontDrive.getCurrentPosition());
            telemetry.addData("Front Right", robot.rightFrontDrive.getCurrentPosition());
            telemetry.addData("Back Left", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("Back Right", robot.rightRearDrive.getCurrentPosition());

            telemetry.addData("Arm", robot.lift.getCurrentPosition()); // 0 - 2235

            telemetry.addData("Extend", robot.exten.getCurrentPosition());

            telemetry.update();

        }
    }
}
