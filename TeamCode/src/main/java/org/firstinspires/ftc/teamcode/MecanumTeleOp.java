package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override

    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        hardware.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.verticalLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntegratingGyroscope gyro;
        NavxMicroNavigationSensor navxMicro;
        ElapsedTime timer = new ElapsedTime();
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "gyro");
        gyro = (IntegratingGyroscope) navxMicro;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double botheading = angles.firstAngle;
            telemetry.addData("Heading", formatAngle(angles.angleUnit, botheading));
//                    .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
//                    .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
//                    .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle))

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double rotX = x * Math.cos(-botheading) - y * Math.sin(-botheading);
            double rotY = x * Math.sin(-botheading) + y * Math.cos(-botheading);
            rotX *= 1.1; // Counteract imperfect strafing

            telemetry.addLine()
                    .addData("rotX", rotX)
                    .addData("rotY", rotY);
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            hardware.frontLeft.setPower(frontLeftPower / 2);
            hardware.backLeft.setPower(backLeftPower / 2);
            hardware.frontRight.setPower(frontRightPower / 2);
            hardware.backRight.setPower(backRightPower / 2);
            /*if(gamepad2.dpad_up){
                hardware.verticalLift.setPower(0.5);

            }
            else if(gamepad2.dpad_down){
                hardware.verticalLift.setPower(-0.5);
            }
            else{
                hardware.verticalLift.setPower(0.0);
            }*/
            lift(hardware);
            int verticalPosition = hardware.encoderLift.getCurrentPosition();
            telemetry.addData("Vertical position",verticalPosition);
            telemetry.addData("fl power", frontLeftPower);
            telemetry.addData("fr power", frontRightPower);
            telemetry.addData("bl power", backLeftPower);
            telemetry.addData("br power", backRightPower);
            telemetry.update();
        }

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    @SuppressLint("DefaultLocale")
    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /////////////////////////////////////////////
    // lifts the vertical slides to a target position
    private void targetLift(Hardware hardware , int targetPosition){

        hardware.verticalLift.setTargetPosition(targetPosition);
        hardware.verticalLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalLift.setPower(0.5);
        while (true){
            int verticalPosition = hardware.encoderLift.getCurrentPosition();
            if (Math.abs (verticalPosition-targetPosition) < 5){
                hardware.verticalLift.setPower(0);
                break;
            }
        }
        hardware.verticalLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    private void lift(Hardware hardware ) {

        //Hardware hardware = new Hardware(hardwareMap);
        int verticalPosition = hardware.encoderLift.getCurrentPosition();

        if (gamepad2.dpad_up) {
            hardware.verticalLift.setPower(0.5);
        } else if (gamepad2.dpad_down) {
            hardware.verticalLift.setPower(-0.5);
        } else {
            hardware.verticalLift.setPower(0.0);
        }

        if (gamepad2.a) {
            targetLift(hardware, 360);

        }
    }
}