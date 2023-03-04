package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "MotorTesting", group = "Challenge")
public class MotorSpeedTest extends OpMode {
    private DcMotorEx motor1, motor2;
    long oldTime, currTime;
    int motor1InitTicks, motor2InitTicks;
    int motor1TicksPerSecond, motor2TicksPerSecond;
    @Override
    public void init() {
        motor1 = this.hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = this.hardwareMap.get(DcMotorEx.class, "motor2");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setPower(1);
        motor2.setPower(1);
        currTime = System.nanoTime();
        telemetry.update();
    }

    @Override
    public void loop() {
        if(currTime % 1e9 == 0)
        {
            motor1TicksPerSecond = motor1.getCurrentPosition()-motor1InitTicks;
            motor2TicksPerSecond = motor2.getCurrentPosition()-motor2InitTicks;
            motor1InitTicks = motor1.getCurrentPosition();
            motor2InitTicks = motor2.getCurrentPosition();
        }
        currTime = System.nanoTime();
        telemetry.addData("motor1 ticks per second:\t", motor1TicksPerSecond);
        telemetry.addData("motor1 ticks per second:\t", motor2TicksPerSecond);
    }
}
