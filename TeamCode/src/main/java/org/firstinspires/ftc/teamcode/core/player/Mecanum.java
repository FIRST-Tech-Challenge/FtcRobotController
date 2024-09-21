package org.firstinspires.ftc.teamcode.core.player;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.Drive;

import java.util.Dictionary;
import java.util.Hashtable;

public class Mecanum {
    static LinearOpMode _linearOpMode;

    static double _max_govenor = 1.0;
    static double _min_govenor = 0.5;
    static final long _govenor_sleep_delay = 350;

    static double _govenor = _max_govenor;

    public static void initialize(LinearOpMode linearOpMode){
        _linearOpMode = linearOpMode;
        Drive.initialize(linearOpMode.hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    static boolean setGovenor(float isTriggered){
        if(isTriggered != 0){
            if (_govenor == _max_govenor) _govenor = _min_govenor;
            else _govenor = _max_govenor;
            return true;
        }

        return false;
    }

    static String getGovenorPercentage() {
        return (_govenor * 100) + "%";
    }

    static Dictionary<String, Double> getPowers(double vertical, double horizontal, double pivot){
        Dictionary<String, Double> powers = new Hashtable<>();
        powers.put("flp", (pivot + vertical + horizontal) * _govenor);
        powers.put("frp", (-pivot + (vertical - horizontal)) * _govenor);
        powers.put("rlp", (pivot + (vertical - horizontal)) * _govenor);
        powers.put("rrp", (-pivot + vertical + horizontal) * _govenor);

        return powers;
    }

    public static void drive() {
        if(setGovenor(_linearOpMode.gamepad1.left_trigger)) _linearOpMode.sleep(_govenor_sleep_delay);

        _linearOpMode.telemetry.addData("Throttle", getGovenorPercentage());

        Dictionary<String, Double> powers = getPowers(_linearOpMode.gamepad1.right_stick_y,
                -_linearOpMode.gamepad1.right_stick_x, -_linearOpMode.gamepad1.left_stick_x);
        Drive.setPower(powers.get("flp"), powers.get("frp"), powers.get("rlp"), powers.get(("rrp")));
        }
}
