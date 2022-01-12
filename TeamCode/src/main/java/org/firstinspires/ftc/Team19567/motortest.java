
package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class motortest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private double motor1Power;
    private double motor2Power;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motor1 = hardwareMap.get(DcMotor.class, "leftBack");
        motor2 = hardwareMap.get(DcMotor.class,"leftFront");

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);

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

        motor1Power = Range.clip(gamepad1.left_stick_y, -1.0, 1.0);
        motor2Power = Range.clip(gamepad1.right_stick_y,-1.0,1.0);

        motor1.setPower(motor1Power);
        motor2.setPower(motor2Power);

        // send telemetry
        telemetry.addData("Status","Running");
        telemetry.addData("Runtime", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "motor1(%.2f), motor2(%.2f)", motor1,motor2);
    }

    @Override
    public void stop() {
        telemetry.addData("Status","Stopped");
    }

}
