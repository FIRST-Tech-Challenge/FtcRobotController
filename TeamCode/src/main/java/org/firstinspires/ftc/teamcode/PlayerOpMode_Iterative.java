package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Config;
import org.firstinspires.ftc.teamcode.core.Drive;

@TeleOp(name="Driver")
//@Disabled
public class PlayerOpMode_Iterative extends OpMode
{
    Drive _drive;
    ElapsedTime _runtime = new ElapsedTime();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        _drive = new Drive();
        _drive.playerInit(hardwareMap);

        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() { }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.clearAll();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Runtime", _runtime.milliseconds());
        telemetry.addData("IsTouching", _drive.isTouching());
        telemetry.addData("Distance (cm)", _drive.getDistance());
        telemetry.update();

        mecanumDrive();

        if(BuildConfig.DEBUG) {
            if (_runtime.milliseconds() > 500) {
                _runtime.reset();
                Log.i(Config.LOG_TAG, "IsTouching: " + _drive.isTouching());
                Log.i(Config.LOG_TAG, "Distance: " + _drive.getDistance());
            }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() { }

    private void mecanumDrive() {
        double vertical = gamepad1.right_stick_y;
        double horizontal = -gamepad1.right_stick_x;
        double pivot = gamepad1.left_stick_x;

        _drive.setPower(
                (pivot + vertical + horizontal),
                (-pivot + (vertical - horizontal)),
                (pivot + (vertical - horizontal)),
                (-pivot + vertical + horizontal)
        );
//        Dictionary<String, Double> powers = drive.getPowers();
//        Enumeration<String> keys = powers.keys();
//        while (keys.hasMoreElements()) {
//            String key = keys.nextElement();
//            Log.i(Config.LOG_TAG, key + ": " + powers.get(key));
//        }
    }
}