
package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="motor_test", group="Iterative Opmode")
public class motortest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private TouchSensor limitSwitch = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motor1 = hardwareMap.get(DcMotor.class, "left_drive");
        motor2 = hardwareMap.get(DcMotor.class,"right_drive");
        limitSwitch = hardwareMap.get(TouchSensor.class,"limitSwitch");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status","Awaiting Start");
    }

    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("Status","OpMode Started");
    }

    @Override
    public void loop() {
        double motor1Power = Range.clip(gamepad1.left_stick_y, -1.0, 1.0);
        double motor2Power = Range.clip(gamepad1.right_stick_y,-1.0,1.0);

        if(limitSwitch.isPressed()) {
            telemetry.addData("Limit Switch","limitSwitch is pressed");
            motor1Power = 1.0;
            motor2Power = -1.0;
        }

        motor1.setPower(motor1Power);
        motor2.setPower(motor2Power);

        // send telemetry
        telemetry.addData("Status","Running");
        telemetry.addData("Runtime", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "motor1(%.2f), motor2(%.2f)", motor1Power,motor2Power);
    }

    @Override
    public void stop() {
        telemetry.addData("Status","Stopped");
    }
}
