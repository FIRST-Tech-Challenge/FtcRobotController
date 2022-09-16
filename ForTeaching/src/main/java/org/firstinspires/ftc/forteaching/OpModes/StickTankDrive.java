package org.firstinspires.ftc.forteaching.OpModes;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.forteaching.TankDriveDemo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.forteaching.TankDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "StickTankDrive")

public class StickTankDrive extends OpMode {
    private static final double DEAD_ZONE = 0.1;
    private TankDriveDemo tankDrive;
    private DcMotorEx motorL;
    private DcMotorEx motorR;
    private Servo servo;
    private RevTouchSensor toucha, touchb;
    private Rev2mDistanceSensor distance;
    private RevColorSensorV3 color;

    @Override
    public void init() {
        motorL = hardwareMap.get(DcMotorEx.class, "motorL");
        motorR = hardwareMap.get(DcMotorEx.class, "motorR");
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        tankDrive = new TankDriveDemo(motorL, motorR);
        servo = hardwareMap.get(Servo.class, "gobilda");
        toucha = hardwareMap.get(RevTouchSensor.class, "touch6");
        touchb = hardwareMap.get(RevTouchSensor.class, "touch7");
        distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance2m");
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        servo = hardwareMap.get(Servo.class, "gobilda");
    }

    @Override
    public void loop() {
        double left, right, srvpos;
        if (Math.abs(gamepad1.left_stick_y) > DEAD_ZONE) {
            left = gamepad1.left_stick_y;
        } else {
            left = 0;
        }
        tankDrive.motorLPower(left);
        if (Math.abs(gamepad1.right_stick_y) > DEAD_ZONE) {
            tankDrive.motorRPower(gamepad1.right_stick_y);
            right = gamepad1.right_stick_y;
        } else {
            right = 0;
        }
        tankDrive.motorRPower(right);
        if (Math.abs(gamepad1.left_stick_x) > DEAD_ZONE) {
            srvpos = (gamepad1.left_stick_x + 1) / 2;
        } else {
            srvpos = 0.5;
        }
        servo.setPosition(srvpos);
        telemetry.addData("L", left);
        telemetry.addData("R", right);
        telemetry.addData("Button", gamepad1.circle ? "O" : "_");
        telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
        telemetry.addData("Color Distance", color.getDistance(DistanceUnit.CM));
        telemetry.addData("Color Light", color.getLightDetected());
        telemetry.addData("Color ARGB", "%x", color.argb());
        telemetry.addData("Color Red", "%x", color.red());
        telemetry.addData("Color Blue", "%x", color.blue());
        telemetry.addData("Color Green", "%x", color.green());
        telemetry.addData("Bump B", touchb.getValue());
        telemetry.addData("Servo Power", srvpos);
        telemetry.update();
    }
    @Override
    public void start() {

    }
}
