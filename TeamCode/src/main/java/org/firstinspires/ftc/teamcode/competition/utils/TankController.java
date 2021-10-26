package org.firstinspires.ftc.teamcode.competition.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;

/**
 * Controls a tank. Used to be called "TankRobot"
 * @author Michael Lachut
 */
public class TankController extends GamepadExtended {

    private final Tank TANK;
    private final Motor SPINNER, ELEVATOR, GRABBER;
    private final Servo TRAPPER;
    private final DistanceSensor DISTANCE_SENSOR;

    public TankController(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, telemetry);
        TANK = new Tank(
                telemetry,
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD),
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.RIGHT_DRIVE_2), DcMotorSimple.Direction.FORWARD),
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.LEFT_DRIVE_1), DcMotorSimple.Direction.FORWARD),
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.LEFT_DRIVE_2), DcMotorSimple.Direction.FORWARD)
        );
        SPINNER = new Motor(telemetry, hardwareMap, "spinner", DcMotorSimple.Direction.FORWARD);
        ELEVATOR = new Motor(telemetry, hardwareMap, "elevator", DcMotorSimple.Direction.FORWARD, 1400, 1.0, 1.0);
        GRABBER = new Motor(telemetry, hardwareMap, "grabber", DcMotorSimple.Direction.FORWARD);
        TRAPPER = new Servo(hardwareMap, "trapper");
        DISTANCE_SENSOR = new DistanceSensor(hardwareMap, "distance");
    }

    @Override
    public void main() {
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.left_stick_x;
        double left    = Range.clip((drive + turn)*100, -100.0, 100.0) ;
        double right   = Range.clip((drive - turn)*100, -100.0, 100.0) ;

        TANK.driveWithoutEncoder((int) -right, (int) -left);

        /*
        if ((gamepad2.left_stick_y >= 0.25 | gamepad2.left_stick_y <= -0.25) && priority.f2(false)) {
            spinner.setPower(gamepad2.left_stick_y);
        }
        else if (gamepad1.right_trigger >= 0.25) { spinner.setPower(gamepad1.left_stick_y); }
        */
    }

    public Tank getTank() {
        return TANK;
    }

}
