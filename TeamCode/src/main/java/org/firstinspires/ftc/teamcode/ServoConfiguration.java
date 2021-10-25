package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;

// Utilizes FtcDashboard
@Config
class ServoConfigValue {
    public static double servoValue = 0.27;
    final Servo servo;

    ServoConfigValue(@NonNull HardwareMap hardwareMap) {
        //Put the name of the servo you want to config here.
        this.servo = hardwareMap.get(Servo.class, "armServo");
    }

    void update() {
        servo.setPosition(servoValue);
    }
}

public class ServoConfiguration extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final ServoConfigValue arm = new ServoConfigValue(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            arm.update();
            telemetry.addData("servoPower",arm.servo.getPosition());
            telemetry.update();
        }
    }
}
