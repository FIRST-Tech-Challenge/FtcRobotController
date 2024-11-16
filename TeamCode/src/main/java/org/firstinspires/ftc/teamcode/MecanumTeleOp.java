package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    private Hardware hardware;
    double Wristpos = 0.28;
    double Twistpos = 0.17;
    double VerticalSlideSpeed = 0.75;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        hardware.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.verticalSlide.setTargetPosition(0);
        hardware.arm.setTargetPosition(0);
        armTargetPosDeg = 0.0;
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.2);
        hardware.wrist.setPosition(0.28);
        hardware.twist.setPosition(Twistpos);
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

        waitForStart();
        if (isStopRequested()) return;

        double yaw_offset = 0.0;
        while (opModeIsActive()) {

            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (gamepad1.back) {
                yaw_offset = angles.firstAngle;
            }
            double botheading = angles.firstAngle - yaw_offset;
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
            wrist();
            servoMoves();
            twist();
            lift(hardware);
            if (gamepad2.y) {
                ScoreHighBasket(hardware);
            }
            if(gamepad2.x){
                PickUpYellow(hardware);
            }
            arm(hardware);
            int verticalPosition = hardware.encoderVerticalSlide.getCurrentPosition();
            telemetry.addData("Vertical position", verticalPosition);
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

    int maxVerticalLiftTicks = 2300;
    int minVerticalLiftTicks = 0;
    int highChamberTicks = 790;
    int highBasketTicks = 2180;

    // lifts the vertical slides to a target position in ticks
    private void targetLift(Hardware hardware, int targetPosition) {

        hardware.verticalSlide.setTargetPosition(targetPosition);
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);
        ElapsedTime Timer = new ElapsedTime();
        double timeoutSeconds = 3.0;
        int allowedErrorTicks = 5;
        while (Timer.time() < timeoutSeconds) {
            int verticalPosition = hardware.encoderVerticalSlide.getCurrentPosition();
            if (Math.abs(verticalPosition - targetPosition) < allowedErrorTicks) {
                hardware.verticalSlide.setPower(0);
                break;
            }
            arm(hardware);
        }
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    int maintainHeightTicks = 0;

    private void lift(Hardware hardware) {

        //Hardware hardware = new Hardware(hardwareMap);
        int verticalPosition = hardware.encoderVerticalSlide.getCurrentPosition();


        if (gamepad2.dpad_up && verticalPosition < maxVerticalLiftTicks) {
            hardware.verticalSlide.setPower(VerticalSlideSpeed);
            hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            maintainHeightTicks = verticalPosition;
            return;
        }

        if (gamepad2.dpad_down && verticalPosition > minVerticalLiftTicks) {
            hardware.verticalSlide.setPower(-VerticalSlideSpeed);
            hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            maintainHeightTicks = verticalPosition;
            return;
        }

        if (gamepad2.b) {
            targetLift(hardware, highChamberTicks);
            maintainHeightTicks = highChamberTicks;
        }

        if (gamepad2.a) {
            targetLift(hardware, 0);
            maintainHeightTicks = 0;
        }
        if (verticalPosition < 47 && maintainHeightTicks < 47) {
            hardware.verticalSlide.setPower(0);
            hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.verticalSlide.setPower(VerticalSlideSpeed);
            hardware.verticalSlide.setTargetPosition(maintainHeightTicks);
        }
    }



    double armTargetPosDeg = 0.0;
    int liftMinClearanceTicks = 350;

    private static int deg2arm(double degrees) {
        return (int) (degrees / 360.0 * spinTickPerRev);
    }

    private double getArmPosDeg() {
        double rotations = hardware.arm.getCurrentPosition() / spinTickPerRev;
        // 0 = straight down
        return rotations * 360.0;
    }

    private boolean checkedArmGoto(double degrees) {
        if (hardware.encoderVerticalSlide.getCurrentPosition() < liftMinClearanceTicks) {
            double current = getArmPosDeg();
            if (degrees > 5 && degrees < 35) return false;
            if (current < 5 && degrees >= 35) return false;
            if (current > 35 && degrees < 5) return false;
        }
        armTargetPosDeg = degrees;
        return true;
    }

    static final double spinTickPerRev = 751.8;

    private void arm(Hardware hardware) {
        // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        // 537.7 ppr
        DcMotor arm = hardware.arm;
        double stick_pos = -gamepad2.right_stick_y;
        double rotations = arm.getCurrentPosition() / spinTickPerRev;
        double degrees = rotations * 360.0; // 0 = straight down
        // Negative: towards front;
        // Positive: towards back.
        // Exclusion zone 0 to -25deg whe lift < 6in.
        boolean emerg = false;
        if (hardware.encoderVerticalSlide.getCurrentPosition() <= liftMinClearanceTicks) {
            // get outta there
            if (stick_pos > 0.7 && (armTargetPosDeg <= 5 || (armTargetPosDeg >= 35 && armTargetPosDeg <= 110))) {
                armTargetPosDeg += 1;
            }
            if (stick_pos < -0.7 && (armTargetPosDeg >= 35 || (armTargetPosDeg >= -110 && armTargetPosDeg <= 5))) {
                armTargetPosDeg -= 1;
            }

            if (armTargetPosDeg > 5 && armTargetPosDeg < 12.5) {
                emerg = true;
                armTargetPosDeg = 5;
            } else if (armTargetPosDeg >= 12.5 && armTargetPosDeg < 35) {
                emerg = true;
                armTargetPosDeg = 35;
            }
        } else {
            // Full* clearance
            if (stick_pos > 0.7 && armTargetPosDeg <= 110) {
                armTargetPosDeg += 1;
            }
            if (stick_pos < -0.7 && armTargetPosDeg >= -110) {
                armTargetPosDeg -= 1;
            }
        }
        arm.setTargetPosition(deg2arm(armTargetPosDeg));
        arm.setPower(emerg ? 1.0 : 0.3);
        telemetry.addData("arm deg", degrees);
    }
    public void twist() {
        if(gamepad2.left_stick_x>=0.5 && gamepad2.left_stick_y>=-0.25 && gamepad2.left_stick_y<=0.25){
            Twistpos+=0.01;
            hardware.twist.setPosition(Twistpos);
        } else if (gamepad2.left_stick_x<=-0.5 && gamepad2.left_stick_y>=-0.25 && gamepad2.left_stick_y<=0.25){
            Twistpos-=0.01;
            hardware.twist.setPosition(Twistpos);
        }
        telemetry.addData("Twist Position",Twistpos);

    }
    public void wrist() {
        if(gamepad2.left_stick_y>=0.5 && gamepad2.left_stick_x>=-0.25 && gamepad2.left_stick_x<=0.25){
            Wristpos += 0.01;
            hardware.wrist.setPosition(Wristpos);
        } else if (gamepad2.left_stick_y<=-0.5 && gamepad2.left_stick_x>=-0.25 && gamepad2.left_stick_x<=0.25){
            Wristpos-= 0.01;
            hardware.wrist.setPosition(Wristpos);
        }

        telemetry.addData("Wrist Position",hardware.wrist.getPosition());
    }
    public void servoMoves(){
        Servo servo = hardwareMap.get(Servo.class,"claw");
        final double open = 0.02;
        final double close = 0.55;
        if(gamepad2.left_bumper) {
            servo.setPosition(open);
        } else if (gamepad2.right_bumper) {
            servo.setPosition(close);

        }
    }
   //////////////////////////////////////////////////////////////////////////////////////////////////
    public void ScoreHighBasket(Hardware hardware)  {
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);
        hardware.verticalSlide.setTargetPosition(highBasketTicks);
        maintainHeightTicks = highBasketTicks;
        sleep(2000);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.5);
        hardware.arm.setTargetPosition(222);
        sleep(500);
        hardware.wrist.setPosition(0.94);
        sleep(500);
        hardware.claw.setPosition(0.02);
        sleep(500);
        hardware.claw.setPosition(0.55);
        sleep(100);
        hardware.wrist.setPosition(0.28);
        sleep(500);
        hardware.arm.setTargetPosition(0);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(0);
        maintainHeightTicks = 0;
    }
    public void PickUpYellow(Hardware hardware){
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);
        hardware.verticalSlide.setTargetPosition(224);
        maintainHeightTicks = 224;
        sleep(500);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.5);
        hardware.arm.setTargetPosition(67);
        sleep(500);
        hardware.wrist.setPosition(0.94);
        sleep(500);
        hardware.claw.setPosition(0.02);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(110);
        maintainHeightTicks = 25;
        sleep(500);
        hardware.claw.setPosition(0.55);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(200);
        maintainHeightTicks = 200;
        sleep(500);
        hardware.wrist.setPosition(0.28);
        sleep(500);
        hardware.arm.setTargetPosition(0);
        sleep(500);

    }
}
