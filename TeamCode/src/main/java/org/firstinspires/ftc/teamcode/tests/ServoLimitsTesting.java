package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class ServoLimitsTesting extends LinearOpMode {
    ServoImplEx servo1;
    ServoImplEx servo2;
//    CRServoImpl servocr1;
//    CRServoImpl servocr2;

    //Basket 0.01 - 0.48
    //Claw
    //Arm 0 - 0.925
    //FrontSl 0.05 -

    Telemetry dahsboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    public static double max = 0.2;
    public static double min = 0.1;

//    double last_min=min, last_max=max;

    public static double pos = 0.2;
    double step = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(ServoImplEx.class, "claw");
//        servo2 = hardwareMap.get(ServoImplEx.class, "frontSlL");

        waitForStart();

        while(opModeIsActive()) {
//            if(gamepad1.dpad_up && pos < (1-step)) pos += step;
//
//            if(gamepad1.dpad_down && pos > step) pos -= step;

            if(gamepad1.dpad_up) servo1.setPosition(max);
            else if(gamepad1.dpad_down) servo1.setPosition(min);

            dahsboardTelemetry.addData("Smth", "");

//            servocr1.setPower(gamepad1.left_stick_y);
//            servocr2.setPower(-gamepad1.left_stick_y);
//            servo1.setPosition(pos);
//            servo2.setPosition(1-pos);
//            sleep(30);
        }
    }
}
