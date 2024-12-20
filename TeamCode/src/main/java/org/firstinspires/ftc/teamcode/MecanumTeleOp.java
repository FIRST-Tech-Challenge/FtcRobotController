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
    double VerticalSlideSpeed = 0.75;
    double ClawFrontPos = 0.5;
    double ClawFlipPos = 1.0;

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
        hardware.arm.setPower(Hardware.ARM_POWER);
        hardware.wrist.setPosition(0.28);
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

            double maxSpeed = 1.0;
            double slowSpeed = 0.5;
            double currentSpeed = maxSpeed;
            if (gamepad1.left_bumper) {
                currentSpeed = slowSpeed;
            }
            hardware.frontLeft.setPower(frontLeftPower * currentSpeed);
            hardware.backLeft.setPower(backLeftPower * currentSpeed);
            hardware.frontRight.setPower(frontRightPower * currentSpeed);
            hardware.backRight.setPower(backRightPower * currentSpeed);
            wrist();
            trasfer(hardware);
            servoMoves();
            stepper(hardware);
            lift(hardware);
            HSlide(hardware);

            if (gamepad2.y) {
                ScoreHighBasket(hardware);
            }
            if (gamepad2.b) {
                specimenWallPick(hardware);
            }
            if (gamepad2.dpad_left) {
                score(hardware);
            }
            if (gamepad1.right_trigger > 0.5) {
                /* Horizontalpick(hardware);*/
                Flipin(hardware);
            }
            if (gamepad1.left_trigger > 0.5) {
                Flipout(hardware);
            }
            arm(hardware);
            int verticalPosition = hardware.encoderVerticalSlide.getCurrentPosition();
            telemetry.addData("Wrist Position", hardware.wrist.getPosition());
            telemetry.addData("Claw Position", hardware.claw.getPosition());
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

    private double getArmPosDeg() {
        double rotations = hardware.arm.getCurrentPosition() / Hardware.spinTickPerRev;
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

    private void arm(Hardware hardware) {
        // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        // 537.7 ppr
        DcMotor arm = hardware.arm;
        double stick_pos = -gamepad2.right_stick_y;
        double rotations = arm.getCurrentPosition() / Hardware.spinTickPerRev;
        double degrees = rotations * 360.0; // 0 = straight down
        // Negative: towards front;
        // Positive: towards back.
        // Exclusion zone 0 to -25deg whe lift < 6in.

        // Full* clearance
        if (stick_pos > 0.7 && armTargetPosDeg <= 110) {
            armTargetPosDeg += 1;
        }
        if (stick_pos < -0.7 && armTargetPosDeg >= -110) {
            armTargetPosDeg -= 1;
        }

        arm.setTargetPosition(Hardware.deg2arm(armTargetPosDeg));
        arm.setPower(0.3);
        telemetry.addData("arm deg", degrees);


    }
    public void horizontalslidein(){
    double overrotateamount = 0.05;
        hardware.horizontalLeft.setPosition(Hardware.LEFT_SLIDE_IN+overrotateamount);
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_IN-overrotateamount);
        sleep(1000);//this shoud be reduced and 200 not enough so between 200 & 1000
        hardware.horizontalLeft.setPosition(Hardware.LEFT_SLIDE_IN);
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_IN);
    }

    public void wrist() {
        if (gamepad2.left_stick_y >= 0.5 && gamepad2.left_stick_x >= -0.25 && gamepad2.left_stick_x <= 0.25) {
            Wristpos += 0.01;
            hardware.wrist.setPosition(Wristpos);

        } else if (gamepad2.left_stick_y <= -0.5 && gamepad2.left_stick_x >= -0.25 && gamepad2.left_stick_x <= 0.25) {
            Wristpos -= 0.01;
            hardware.wrist.setPosition(Wristpos);
        }

        telemetry.addData("Wrist Position", hardware.wrist.getPosition());
    }

    public void servoMoves() {
        Servo servo = hardwareMap.get(Servo.class, "claw");
        if (gamepad2.left_bumper) {
            servo.setPosition(Hardware.CLAW_CLOSE);
        } else if (gamepad2.right_bumper) {
            servo.setPosition(Hardware.CLAW_OPEN);

        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    public void ScoreHighBasket(Hardware hardware) {
        hardware.claw.setPosition(0.02);
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
        sleep(700);
        hardware.claw.setPosition(Hardware.CLAW_OPEN);
        sleep(100);
        hardware.wrist.setPosition(0.28);
        sleep(500);
        hardware.arm.setTargetPosition(0);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(0);
        maintainHeightTicks = 0;
    }

    public void PickUpYellow(Hardware hardware) {
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
        hardware.claw.setPosition(Hardware.CLAW_CLOSE);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(110);
        maintainHeightTicks = 25;
        sleep(500);
        hardware.claw.setPosition(Hardware.CLAW_OPEN);
        sleep(500);
        hardware.verticalSlide.setTargetPosition(200);
        maintainHeightTicks = 200;
        sleep(500);
        hardware.wrist.setPosition(0.28);
        sleep(500);
        hardware.arm.setTargetPosition(0);
        sleep(500);
    }


    private void stepper(Hardware hardware) {
        if (gamepad1.dpad_left) {
            ClawFrontPos = Hardware.FRONT_OPEN;
        }
        if (gamepad1.dpad_right) {
            ClawFrontPos = Hardware.FRONT_CLOSE;
        }
        if (gamepad1.dpad_down) {
            ClawFlipPos -= 0.01;
            if(ClawFlipPos<0){
                ClawFlipPos = 0.0;
            }
        }
        if (gamepad1.dpad_up) {
            ClawFlipPos += 0.01;
            if(ClawFlipPos>1){
                ClawFlipPos = 1.0;
            }
        }
        hardware.clawFlip.setPosition(ClawFlipPos);
        hardware.clawFront.setPosition(ClawFrontPos);
        // clawFront close is 0
        //clawFront open is 0.27

        telemetry.addData("FrontClawPos", ClawFrontPos);
        telemetry.addData("FlipClawPos", ClawFlipPos);
    }

    public void HSlide(Hardware hardware) {

        if (gamepad1.y) {
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_OUT);
        hardware.horizontalLeft.setPosition(Hardware.LEFT_SLIDE_OUT);
        }
        if (gamepad1.a) {
        horizontalslidein();
        }

        telemetry.addData("Horizontal Left Position", hardware.horizontalLeft.getPosition());
        telemetry.addData("Horizontal Position", hardware.horizontalSlide.getPosition());
    }

    public void specimenWallPick(Hardware hardware) {
        hardware.claw.setPosition(Hardware.CLAW_OPEN);
        sleep(1000);
        hardware.wrist.setPosition(Hardware.WRIST_UP);
        sleep(1000);
        hardware.arm.setTargetPosition(45);
        armTargetPosDeg = 45;
        sleep(500);
        hardware.claw.setPosition(Hardware.CLAW_CLOSE);
        sleep(1000);
        hardware.verticalSlide.setTargetPosition(300);
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);
        maintainHeightTicks = 300;
        sleep(1000);
        hardware.wrist.setPosition(Hardware.WRIST_BACK);
        sleep(1000);
        hardware.arm.setTargetPosition(10);
        armTargetPosDeg = 10;
        sleep(1000);
        hardware.verticalSlide.setTargetPosition(0);
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);
        maintainHeightTicks = 0;
    }

    private void score(Hardware hardware) {
        double clawclose = Hardware.CLAW_CLOSE;

        hardware.claw.setPosition(clawclose);
        hardware.verticalSlide.setTargetPosition(710);
        hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.verticalSlide.setPower(VerticalSlideSpeed);
        maintainHeightTicks = 710;
        sleep(1000);
        hardware.arm.setTargetPosition(Hardware.deg2arm(-99));
        armTargetPosDeg = -99;
        sleep(1000);
        hardware.wrist.setPosition(1);

    }

    public void Horizontalpick(Hardware hardware) {
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_OUT);
        hardware.horizontalLeft.setPosition(Hardware.LEFT_SLIDE_OUT);
        sleep(500);
        hardware.clawFlip.setPosition(Hardware.FLIP_DOWN);
        sleep(500);
        hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
        sleep(500);
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
        ClawFrontPos = Hardware.FRONT_CLOSE;
        sleep(500);
        hardware.clawFlip.setPosition(Hardware.FLIP_UP);
        sleep(500);
        horizontalslidein();
        sleep(500);
    }

    public void Flipout(Hardware hardware) {
        hardware.horizontalSlide.setPosition(Hardware.RIGHT_SLIDE_OUT);
        hardware.horizontalLeft.setPosition(Hardware.LEFT_SLIDE_OUT);
        sleep(500);
        hardware.clawFlip.setPosition(Hardware.FLIP_DOWN);
        ClawFlipPos = Hardware.FLIP_DOWN;
        sleep(500);
    }

    public void Flipin(Hardware hardware) {
        double fliponethird = 0.66;
        hardware.clawFlip.setPosition(fliponethird);
        ClawFlipPos = fliponethird;
        sleep(500);
        horizontalslidein();
        sleep(500);
        hardware.clawFlip.setPosition(Hardware.FLIP_UP);
        ClawFlipPos = Hardware.FLIP_UP;
        sleep(500);
    }

    boolean lastX = false;

    public void trasfer(Hardware hardware){
        boolean x = gamepad2.x;
        if (x && !lastX) {
            hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
            ClawFrontPos = Hardware.FRONT_CLOSE;
            hardware.claw.setPosition(Hardware.CLAW_OPEN);
            sleep(500);
            hardware.wrist.setPosition(0);
            sleep(500);
            hardware.arm.setTargetPosition(-28);
            armTargetPosDeg = -8;
            sleep(1000);
            hardware.claw.setPosition(Hardware.CLAW_CLOSE);
            sleep(500);
            hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
            sleep(500);
            hardware.arm.setTargetPosition(0);
            armTargetPosDeg=0;
            sleep(500);
            hardware.wrist.setPosition(0.28);
            ClawFrontPos = Hardware.FRONT_OPEN;
        }
    }
}

