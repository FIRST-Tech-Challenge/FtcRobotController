package org.firstinspires.ftc.teamcode;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class speedTestOpMode extends OpMode {
    private VoltageSensor voltageSensor;
    private DcMotorEx inTakeMotor = null;
    Timer timer = new Timer();
    double motorSpeed = 10;
    double lowestPower = 100;
    double presentVoltage = 1000000000;

    boolean isStopwatchRunning = false;

    @Override
    public void init() {
        inTakeMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        inTakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        inTakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
                motorSpeed = motorSpeed + 10;
        }
        if (gamepad1.aWasPressed()) {
            motorSpeed = motorSpeed - 10;
        }
        presentVoltage = voltageSensor.getVoltage();
        if (gamepad1.x) {
            inTakeMotor.setVelocity(motorSpeed, AngleUnit.DEGREES);
        } else if (motorSpeed == 0) {
            inTakeMotor.setVelocity(motorSpeed, AngleUnit.DEGREES);
        }
        if(lowestPower > presentVoltage) {
            lowestPower = presentVoltage;
        }
        if (isStopwatchRunning) {
            telemetry.addData("Timer", timer.milliseconds() + "ms");
        }

        telemetry.addData("Motor velocity",motorSpeed);
        telemetry.addData("motor Rpm",inTakeMotor.getVelocity());
        telemetry.addData("lowest battery power", lowestPower);
        telemetry.addData("velocity in rpm",inTakeMotor.getVelocity() * 60);
            telemetry.update();

        /** if (gamepad1.dpad_up) {
         *   if (!isStopwatchRunning) {
         *       timer.reset();
         *       isStopwatchRunning = true;
         *   }
         *   else isStopwatchRunning = false;
         *   }
         */
        if (gamepad1.dpad_up){
            motorSpeed = 0;
        }
    }
}