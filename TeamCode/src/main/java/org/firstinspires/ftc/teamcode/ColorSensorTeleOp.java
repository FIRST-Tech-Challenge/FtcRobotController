
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ColorSensorTeleOp
        extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        Servo lightLeft = hardwareMap.get(Servo.class, "lightLeft");
        Servo lightRight = hardwareMap.get(Servo.class, "lightRight");
        lightRight.setPosition(0);
        lightLeft.setPosition(0);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ColorSensor clawColor = hardwareMap.get(ColorSensor.class, "clawColor");


        waitForStart();


        if (isStopRequested()) return;
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower / 2);
            backLeft.setPower(backLeftPower / 2);
            frontRight.setPower(frontRightPower / 2);
            backRight.setPower(backRightPower / 2);
            double red = clawColor.red();
            double blue = clawColor.blue();
            double green = clawColor.green();

            if (blue - green > 100 && blue - red > 100) {
                lightRight.setPosition(Hardware.LAMP_BLUE);
                lightLeft.setPosition(Hardware.LAMP_BLUE);
                telemetry.addLine("blue");
            }
            else if (red - blue > 100 && red - green > 100) {
                telemetry.addLine("red");
                lightRight.setPosition(Hardware.LAMP_RED);
                lightLeft.setPosition(Hardware.LAMP_RED);
            }
            else if (green - blue > 100 && green - red > 100 && red >= 350) {
                telemetry.addLine("yellow");
                lightRight.setPosition(Hardware.LAMP_YELLOW);
                lightLeft.setPosition(Hardware.LAMP_YELLOW);
            }
            else{
            lightLeft.setPosition(0);
            lightRight.setPosition(0);
            }


            telemetry.addData("amount green", green);
            telemetry.addData("amount red", red);
            telemetry.addData("amount blue", blue);
            telemetry.update();

        }
    }
}


//end class.