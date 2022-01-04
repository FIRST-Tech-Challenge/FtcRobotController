package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import androidx.annotation.NonNull;

@TeleOp
public class ServoConfiguration extends LinearOpMode {

    // Utilizes FtcDashboard
    @Config
    static class ServoConfigValue {
        public static double servoPos = 0.88;
        public static String servoName = "leftTse";
        //0.59 closed, 1 open r
        //0.42 closed, 0 open l
        final Servo servo;

        ServoConfigValue(@NonNull HardwareMap hardwareMap) {
            this.servo = hardwareMap.get(Servo.class, servoName);
        }

        void update() {
            servo.setPosition(servoPos);
        }
        @Config
        static class MotorPos {
            public static int motorPos = 0;
            public static String motorName = "liftMotor";
        }
    }

    protected boolean withMotor = false;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final ServoConfigValue arm = new ServoConfigValue(hardwareMap);
        DcMotor motor = null;
        if (withMotor) {
            motor = hardwareMap.get(DcMotor.class, ServoConfigValue.MotorPos.motorName);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            try { Thread.sleep(100); } catch (InterruptedException ignored) {}
            motor.setTargetPosition(ServoConfigValue.MotorPos.motorPos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
        waitForStart();
        while(opModeIsActive()) {
            arm.update();
            telemetry.addData("servoPos",arm.servo.getPosition());
            if (withMotor) {
                motor.setTargetPosition(ServoConfigValue.MotorPos.motorPos);
                telemetry.addData("motorPos", motor.getCurrentPosition());
            }
            telemetry.update();
        }
    }
}

