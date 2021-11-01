package org.firstinspires.ftc.teamcode.competition.utils;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;

/**
 * Controls a tank. Used to be called "TankRobot"
 * @author Michael Lachut
 */
public class TeleOpTankController extends GamepadExtended {

    private final Tank TANK;
    private final Motor SPINNER, LIFT, GRABBER;
    private final Servo TRAPPER;
    private final DistanceSensor TRAPPER_TRIGGER;

    public TeleOpTankController(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, telemetry);
        TANK = new Tank(
                telemetry,
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD),
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_2), DcMotorSimple.Direction.FORWARD),
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1), DcMotorSimple.Direction.FORWARD),
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_2), DcMotorSimple.Direction.FORWARD)
        );
        SPINNER = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_SPINNER), DcMotorSimple.Direction.FORWARD);
        LIFT = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT), DcMotorSimple.Direction.FORWARD, 1400, 1.0, 1.0);
        GRABBER = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_GRABBER), DcMotorSimple.Direction.FORWARD);
        TRAPPER = hardwareMap.get(Servo.class, hardwareMap.appContext.getString(R.string.HW_TRAPPER));
        TRAPPER_TRIGGER = hardwareMap.get(DistanceSensor.class, hardwareMap.appContext.getString(R.string.HW_TRAPPER_TRIGGER));
    }

    @Override
    public void main() {
        double drive = -gamepad1.left_stick_y;
        double turn =  gamepad1.left_stick_x;
        double left = Range.clip((drive + turn)*100, -100.0, 100.0) ;
        double right = Range.clip((drive - turn)*100, -100.0, 100.0) ;

        TANK.driveWithoutEncoder((int) -right, (int) -left);

        /*
        if ((gamepad2.left_stick_y >= 0.25 | gamepad2.left_stick_y <= -0.25) && priority.f2(false)) {
            spinner.setPower(gamepad2.left_stick_y);
        }
        else if (gamepad1.right_trigger >= 0.25) { spinner.setPower(gamepad1.left_stick_y); }
        */
    }

    @Override
    public void stop() {

    }

    public Tank getTank() {
        return TANK;
    }

}
