package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;

// Utilizes FtcDashboard
@Config
class ServoConfigValue {
    public static double servoPos = 0.27;
    final Servo servo;

    ServoConfigValue(@NonNull HardwareMap hardwareMap) {
        //Put the name of the servo you want to config here.
        this.servo = hardwareMap.get(Servo.class, "armServo");
    }

    void update() {
        servo.setPosition(servoPos);
    }
}
@Config
class MotorPos {
    public static int motorPos = 0;
}

@TeleOp
public class ServoConfiguration extends LinearOpMode {
    protected boolean withMotor = false;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final ServoConfigValue arm = new ServoConfigValue(hardwareMap);
        DcMotor motor = null;
        GamepadEx toolGamepad = null;
        if (withMotor) {
            motor = hardwareMap.get(DcMotor.class, "liftMotor");
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            try { Thread.sleep(100); } catch (InterruptedException ignored) {}
            motor.setTargetPosition(MotorPos.motorPos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
            toolGamepad = new GamepadEx(gamepad2);
        }
        waitForStart();
        while(opModeIsActive()) {
            arm.update();
            telemetry.addData("servoPos",arm.servo.getPosition());
            if (withMotor) {
                motor.setPower(MotorPos.motorPos);
                telemetry.addData("motorPos", motor.getCurrentPosition());
            }
            telemetry.update();
        }
    }
}
