package org.firstinspires.ftc.teamcode.robots.Mechavator;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
@Disabled
@TeleOp(name="Mechavator", group="AAA")

// ****** STOP - This version of Mechavator is retired - use version built on OpenRC

public class Mechavator extends OpMode {

    AnalogInput leftTrack, rightTrack;
    FtcDashboard dashboard;
    DcMotorEx motorLeft, motorRight, motorStick, motorSwing, motorBoom, motorBucket;
    Servo deadManSafety, servoThumb, camPan, camTilt;
    Servo deadManSafety2; //the backup safety is being moved to an independent RC controller
    StickyGamepad stickyGamepad1;
    PIDController leftController, rightController;
    public static PIDCoefficients ACTUATOR_COEFFICIENTS = new PIDCoefficients(-20, 0, 0);
//    public static double TOLERANCE = 0.03;
//    public static double POWER = -1.0;
    public static double ZERO_POSITION = 1.0;
    public static double MAX_OFFSET = 0.65;
    public static double SAFETY_SAFED = 1.0; //position where safety is enabled - servo relaxed
    public static double SAFETY_UNSAFED = .5; //position where safety is disabled - hydraulics are enabled - hold red down to engage
//    public static double MAX_POSITION = 2.67 / 2.0 + 2.67 * 1/6;
    double targetLeftPosition, targetRightPosition;
    double targetStick, targetSwing, targetBoom, targetBucket;
    int offsetStick, offsetSwing, offsetBoom, offsetBucket;
    int rangeStick, rangeSwing, rangeBoom, rangeBucket;
    boolean manual = true;
    boolean homed=false, homing=false, ranging=false, centering=false;
    double homingPower = 0.3;
    double homeTime;
    double leftPosition, rightPosition;

    @Override
    public void init() {
        leftTrack = hardwareMap.analogInput.get("leftinput");
        motorLeft = hardwareMap.get(DcMotorEx.class, "leftmotor");
        rightTrack = hardwareMap.analogInput.get("rightinput");
        motorRight = hardwareMap.get(DcMotorEx.class, "rightmotor");
        motorStick = hardwareMap.get(DcMotorEx.class, "stick");
        motorSwing = hardwareMap.get(DcMotorEx.class, "swing");
        motorBoom = hardwareMap.get(DcMotorEx.class, "boom");
        motorBucket = hardwareMap.get(DcMotorEx.class, "bucket");
        deadManSafety = hardwareMap.get(Servo.class, "deadManSafety");
        camPan = hardwareMap.get(Servo.class, "pan");
        camTilt = hardwareMap.get(Servo.class, "tilt");

        //deadManSafety2 = hardwareMap.get(Servo.class, "backupSafety");
        //servoThumb = hardwareMap.get(Servo.class, "thumb");
        //deadManSafety2 = hardwareMap.get(ServoImplEx.class, "deadManSafety");

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SetJoystickMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //some joystick motors may need to be reversed
        motorStick.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorSwing.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBoom.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBucket.setDirection(DcMotorSimple.Direction.REVERSE);
        camPan.setPosition(.5);
        camTilt.setPosition(.5);

        dashboard = FtcDashboard.getInstance();
        stickyGamepad1 = new StickyGamepad(gamepad1);

        leftController = new PIDController(ACTUATOR_COEFFICIENTS);
        leftController.setInputRange(0, 2.67);
        leftController.setOutputRange(-1.0, 1.0);
        leftController.enable();

        rightController = new PIDController(ACTUATOR_COEFFICIENTS);
        rightController.setInputRange(0, 2.67);
        rightController.setOutputRange(-1.0, 1.0);
        rightController.enable();

        homeTime = futureTime(5);
        homing = true;
        SendTelemetry();

    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        //calibrate the joysticks
        homed=JoysticksCalibrated();
        SendTelemetry();
    }

    static public double map(double value,
                             double istart,
                             double istop,
                             double ostart,
                             double ostop) {
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
    }

    public void SetJoystickMode(DcMotor.RunMode rm){
        motorStick.setMode(rm);
        motorSwing.setMode(rm);
        motorBoom.setMode(rm);
        motorBucket.setMode(rm);
    }

    public void SetJoystickPower(double pwr){
        motorStick.setPower(pwr);
        motorSwing.setPower(pwr);
        motorBoom.setPower(pwr);
        motorBucket.setPower(pwr);
    }

    public void SetJoystickPositions(int target){
        motorStick.setTargetPosition(target);
        motorSwing.setTargetPosition(target);
        motorBoom.setTargetPosition(target);
        motorBucket.setTargetPosition(target);
    }

    public boolean CenterJoysticksFromExtents(){
        //call this only when physically at ranged extents during the calibration routine
        if (!centering) {
            rangeStick = motorStick.getCurrentPosition();
            rangeSwing = motorSwing.getCurrentPosition();
            rangeBoom = motorBoom.getCurrentPosition();
            rangeBucket = motorBucket.getCurrentPosition();
            SetJoystickMode(DcMotor.RunMode.RUN_TO_POSITION);
            SetJoystickPower(.3);
            motorStick.setTargetPosition(rangeStick/2);
            motorSwing.setTargetPosition(rangeSwing/2);
            motorBoom.setTargetPosition(rangeBoom/2);
            motorBucket.setTargetPosition(rangeBucket/2);

        }
        centering = true;

        if(!motorStick.isBusy() && !motorSwing.isBusy() && !motorBoom.isBusy() &&!motorBucket.isBusy())
        {
            //joysticks should now be centered on their range of motion
            SetJoystickMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //motors should all be at a zeroed home position where the joysticks are centered
            //this is our new zero position
            SetJoystickPositions(0);
            SetJoystickMode(DcMotor.RunMode.RUN_TO_POSITION);
            SetJoystickPower(1.0);
            return true;
        }

        return false;

    }

    public boolean JoysticksCalibrated(){
        if (homing) {
            if (System.nanoTime() < homeTime) {
                SetJoystickPower(homingPower);
            }
            else { //enough time should have elapsed for joysticks to stall at their furthest extents (left/up)
                homing = false;
                ranging = true;
                homeTime = futureTime(5);
                //reset the encoders after they've had time to stall at home position
                //would be better to have actual limit switches, but this makes wiring simpler
                SetJoystickMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                SetJoystickMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
            return false;

        }
        if (ranging) {
            if (System.nanoTime() < homeTime) {
                SetJoystickPower(-homingPower);
            }
            else { //enough time should have elapsed for joysticks to stall at their furthest extents (right/down)
                ranging = false;
                homeTime = futureTime(5);
                CenterJoysticksFromExtents();

            }
            return false;

        }

        if(centering)
        {
            if (CenterJoysticksFromExtents()) return true;
        }

        return false;

    }

    @Override
    public void loop() {


        if(stickyGamepad1.back && manual) {
            manual = false;
            targetLeftPosition = ZERO_POSITION;
            targetRightPosition = ZERO_POSITION;
        } else if(stickyGamepad1.back) {
            manual = true;
            targetLeftPosition = ZERO_POSITION;
            targetRightPosition = ZERO_POSITION;
        }

        leftPosition = leftTrack.getVoltage();
        rightPosition = rightTrack.getVoltage();


        if (gamepad1.left_trigger>.5) //hold in for safety disabled
        {
            deadManSafety.setPosition(SAFETY_UNSAFED);
        }
        else {
            deadManSafety.setPosition(SAFETY_SAFED);
        }

        if(manual) {
            targetLeftPosition = map(-gamepad1.left_stick_y, -1.0, 1.0, ZERO_POSITION - MAX_OFFSET, ZERO_POSITION + MAX_OFFSET);
            targetRightPosition = map(-gamepad1.right_stick_y, -1.0, 1.0, ZERO_POSITION - MAX_OFFSET, ZERO_POSITION + MAX_OFFSET);
            targetStick = map(gamepad2.left_stick_y, -1, 1, -rangeStick/2, rangeStick/2);
            targetSwing = map(gamepad2.left_stick_x, -1, 1, -rangeSwing/2, rangeSwing/2);
            targetBoom = map(gamepad2.right_stick_y, -1, 1, -rangeBoom/2, rangeBoom/2);
            targetBucket = map(gamepad2.right_stick_x, -1, 1, -rangeBucket/2, rangeBucket/2);
        }
//        if(!manual) {
            leftController.setPID(ACTUATOR_COEFFICIENTS.p, ACTUATOR_COEFFICIENTS.i, ACTUATOR_COEFFICIENTS.d);
            leftController.setInput(leftPosition);
            leftController.setSetpoint(targetLeftPosition);
            motorLeft.setPower(leftController.performPID());

            rightController.setPID(ACTUATOR_COEFFICIENTS.p, ACTUATOR_COEFFICIENTS.i, ACTUATOR_COEFFICIENTS.d);
            rightController.setInput(rightPosition);
            rightController.setSetpoint(targetRightPosition);
            motorRight.setPower(rightController.performPID());

            motorStick.setTargetPosition((int)targetStick);
            motorSwing.setTargetPosition((int)targetSwing);
            motorBoom.setTargetPosition((int)targetBoom);
            motorBucket.setTargetPosition((int)targetBucket);


//            double leftPower = 0;
//            if (Math.abs(leftPosition - targetLeftPosition) < TOLERANCE)
//                leftPower = 0;
//            else if (leftPosition > targetLeftPosition)
//                leftPower = -POWER;
//            else if (leftPosition < targetLeftPosition)
//                leftPower = POWER;
//            leftMotor.setPower(leftPower);
//
//            double rightPower = 0;
//            if (Math.abs(rightPosition - targetRightPosition) < TOLERANCE)
//                rightPower = 0;
//            else if (rightPosition > targetRightPosition)
//                rightPower = -POWER;
//            else if (rightPosition < targetRightPosition)
//                rightPower = POWER;
//            rightMotor.setPower(rightPower);
//        } else {
//            double leftPower = -gamepad1.left_stick_y;
//            double rightPower = -gamepad1.right_stick_y;
//            leftMotor.setPower(leftPower);
//            rightMotor.setPower(rightPower);
//        }

SendTelemetry();
    }
    void SendTelemetry (){
        telemetry.addData("manual", manual);
        telemetry.addData("left position", leftPosition);
        telemetry.addData("right position", rightPosition);
        telemetry.addData("target left position", targetLeftPosition);
        telemetry.addData("target right position", targetRightPosition);


        TelemetryPacket packet = new TelemetryPacket();
        packet.put("left position", leftPosition);
        packet.put("right position", rightPosition);
        packet.put("target left position", targetLeftPosition);
        packet.put("target right position", targetRightPosition);
        packet.put("manual", manual);
        packet.put("stick target", targetStick);
        packet.put("stick actual", motorStick.getCurrentPosition());
        packet.put("swing target", targetSwing);
        packet.put("swing actual", motorSwing.getCurrentPosition());
        packet.put("boom target", targetBoom);
        packet.put("boom actual", motorBoom.getCurrentPosition());
        packet.put("bucket target", targetBucket);
        packet.put("bucket actual", motorBucket.getCurrentPosition());

        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
        stickyGamepad1.update();
    }
}
