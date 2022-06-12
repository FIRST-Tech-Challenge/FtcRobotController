package org.firstinspires.ftc.teamcode.robots.linearactuator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
@TeleOp(name="AAAAA linear actuator", group="Iterative Opmode")
public class LinearActuatorOpMode extends OpMode {

    AnalogInput leftInput, rightInput;
    FtcDashboard dashboard;
    DcMotorEx leftMotor, rightMotor;
    Servo deadManSafety;
    ServoImplEx deadManSafety2;
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
    boolean manual = true;

    @Override
    public void init() {
        leftInput = hardwareMap.analogInput.get("leftinput");
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftmotor");
        rightInput = hardwareMap.analogInput.get("rightinput");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightmotor");
        deadManSafety = hardwareMap.get(Servo.class, "deadManSafety");
        //deadManSafety2 = hardwareMap.get(ServoImplEx.class, "deadManSafety");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    }

    static public double map(double value,
                             double istart,
                             double istop,
                             double ostart,
                             double ostop) {
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
    }

    @Override
    public void loop() {

//        if(stickyGamepad1.y) {
//            targetLeftPosition = 0;
//        } else if(stickyGamepad1.b) {
//            targetLeftPosition = (1.0 / 3.0) * 2.67;
//        } else if(stickyGamepad1.a) {
//            targetLeftPosition = (2.0 / 3.0) * 2.67;
//        } else if(stickyGamepad1.x) {
//            targetLeftPosition = 2.67;
//        }
//
//        if(stickyGamepad1.dpad_up) {
//            targetRightPosition = 0;
//        } else if(stickyGamepad1.dpad_right) {
//            targetRightPosition = (1.0 / 3.0) * 2.67;
//        } else if(stickyGamepad1.dpad_down) {
//            targetRightPosition = (2.0 / 3.0) * 2.67;
//        } else if(stickyGamepad1.dpad_left) {
//            targetRightPosition = 2.67;
//        }

        if(stickyGamepad1.back && manual) {
            manual = false;
            targetLeftPosition = ZERO_POSITION;
            targetRightPosition = ZERO_POSITION;
        } else if(stickyGamepad1.back) {
            manual = true;
            targetLeftPosition = ZERO_POSITION;
            targetRightPosition = ZERO_POSITION;
        }

        double leftPosition = leftInput.getVoltage();
        double rightPosition = rightInput.getVoltage();
//
//        if(stickyGamepad1.left_bumper)
//            manual = !manual;

        if (gamepad1.b) //hold down for safety disabled
        {
            deadManSafety.setPosition(SAFETY_UNSAFED);
        }
        else {
            deadManSafety.setPosition(SAFETY_SAFED);
        }

        if(manual) {
            targetLeftPosition = map(-gamepad1.left_stick_y, -1.0, 1.0, ZERO_POSITION - MAX_OFFSET, ZERO_POSITION + MAX_OFFSET);
            targetRightPosition = map(-gamepad1.right_stick_y, -1.0, 1.0, ZERO_POSITION - MAX_OFFSET, ZERO_POSITION + MAX_OFFSET);
        }
//        if(!manual) {
            leftController.setPID(ACTUATOR_COEFFICIENTS.p, ACTUATOR_COEFFICIENTS.i, ACTUATOR_COEFFICIENTS.d);
            leftController.setInput(leftPosition);
            leftController.setSetpoint(targetLeftPosition);
            leftMotor.setPower(leftController.performPID());

            rightController.setPID(ACTUATOR_COEFFICIENTS.p, ACTUATOR_COEFFICIENTS.i, ACTUATOR_COEFFICIENTS.d);
            rightController.setInput(rightPosition);
            rightController.setSetpoint(targetRightPosition);
            rightMotor.setPower(rightController.performPID());

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

        telemetry.addData("left position", leftPosition);
        telemetry.addData("right position", rightPosition);
        telemetry.addData("target left position", targetLeftPosition);
        telemetry.addData("target right position", targetRightPosition);
        telemetry.addData("manual", manual);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("left position", leftPosition);
        packet.put("right position", rightPosition);
        packet.put("target left position", targetLeftPosition);
        packet.put("target right position", targetRightPosition);
        packet.put("manual", manual);

        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
        stickyGamepad1.update();
    }
}
