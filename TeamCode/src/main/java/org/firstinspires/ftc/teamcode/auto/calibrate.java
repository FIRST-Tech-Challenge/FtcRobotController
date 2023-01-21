package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.fileUtils.writeToFile;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.BaseOpMode;
import org.firstinspires.ftc.teamcode.config.Hardware2;
@TeleOp(name= "calibrate")
public class calibrate extends BaseOpMode {

    Hardware2 robot = new Hardware2(false);
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        robot.initTeleOpIMU(hardwareMap);
        int initFrontRightTicks = 0;
        int initFrontLeftTicks = 0;
        int initBackRightTicks = 0;
        int initBackLeftTicks = 0;
        int initArmTicks = 0;
        boolean iscalibrating = false;
        while (opModeIsActive()) {
            if (gamepad1.a && !iscalibrating) {
                resetRuntime();
                initFrontLeftTicks = robot.getLeftDrive().getCurrentPosition();
                initBackLeftTicks = robot.getBackLeftDrive().getCurrentPosition();
                initBackRightTicks = robot.getBackRightDrive().getCurrentPosition();
                initFrontRightTicks = robot.getRightDrive().getCurrentPosition();
                initArmTicks = robot.getArm().getCurrentPosition();

                iscalibrating = true;
            } else if (iscalibrating && gamepad1.a) {
                double time = getRuntime();
                telemetry.addData("FR speed: ", (robot.getRightDrive().getCurrentPosition() - initFrontRightTicks)/time);
                telemetry.addData("FL speed: ", (robot.getLeftDrive().getCurrentPosition() - initFrontLeftTicks)/time);
                telemetry.addData("BR speed: ", (robot.getBackRightDrive().getCurrentPosition() - initBackRightTicks)/time);
                telemetry.addData("BL speed: ", (robot.getBackLeftDrive().getCurrentPosition() - initBackLeftTicks)/time);
                telemetry.addData("ARM speed: ", (robot.getArm().getCurrentPosition() - initArmTicks)/time);
                telemetry.addData("Time: ", time);
                telemetry.update();
                String myString = ((robot.getRightDrive().getCurrentPosition() - initFrontRightTicks)/time) + " " + ((robot.getLeftDrive().getCurrentPosition() - initFrontLeftTicks)/time) + " " + ((robot.getBackRightDrive().getCurrentPosition() - initBackRightTicks)/time) + " " + ((robot.getBackLeftDrive().getCurrentPosition() - initBackLeftTicks)/time) + " " + ((robot.getArm().getCurrentPosition() - initArmTicks)/time);
                writeToFile(myString, "movement");
                resetRuntime();
            }

            double drive = gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;
            double leftPower = Range.clip(drive + turn, -0.8, 0.8) ;
            double rightPower = Range.clip(drive - turn, -0.8, 0.8) ;

            robot.frontRightMotor.setPower(rightPower);
            robot.backRightMotor.setPower(rightPower);
            robot.frontLeftMotor.setPower(leftPower);
            robot.backLeftMotor.setPower(leftPower);

            if (gamepad2.left_bumper) {
                robot.getLeftClaw().setPosition(-1);
                robot.getRightClaw().setPosition(1);
            }
            else if (gamepad2.right_bumper) {
                robot.getLeftClaw().setPosition(0.3);
                robot.getRightClaw().setPosition(-0.3);
            }

            if (gamepad2.left_trigger > 0.2) {
                robot.getArm().setPower(-0.7);
            } else if (gamepad2.right_trigger > 0.3) {
                robot.getArm().setPower(0.5);
            } else {
                robot.getArm().setPower(0);
            }


            if (gamepad1.x) {
                robot.frontRightMotor.setPower(1);
                robot.backRightMotor.setPower(-1);
                robot.frontLeftMotor.setPower(-1);
                robot.backLeftMotor.setPower(1);
            } else if (gamepad1.b) {
                robot.frontRightMotor.setPower(-1);
                robot.backRightMotor.setPower(1);
                robot.frontLeftMotor.setPower(1);
                robot.backLeftMotor.setPower(-1);
            }
        }
    }


    @Override
    public Hardware2 getRobot() {
        return this.robot;
    }
}
