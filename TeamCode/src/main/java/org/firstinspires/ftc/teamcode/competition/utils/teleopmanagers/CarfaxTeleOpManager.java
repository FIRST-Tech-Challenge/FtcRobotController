package org.firstinspires.ftc.teamcode.competition.utils.teleopmanagers;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.Carfax;
import org.firstinspires.ftc.teamcode.competition.utils.Motor;
import org.firstinspires.ftc.teamcode.competition.utils.Tank;

public class CarfaxTeleOpManager extends TeleOpManager {

    private final Carfax CARFAX;
    private final Motor SPINNER, LIFT;

    public CarfaxTeleOpManager(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, GamepadFunctions function1, GamepadFunctions function2) {
        super(gamepad1, function1, gamepad2, function2);
        SPINNER = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_SPINNER), DcMotorSimple.Direction.FORWARD);
        LIFT = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT), DcMotorSimple.Direction.FORWARD);
        CARFAX = new Carfax(telemetry, new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD), new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1), DcMotorSimple.Direction.REVERSE));
    }

    /**
     * Show me the Carfax™.
     */
    @Override
    public void main() {
        if(getGamepad1Functions().hasF1()) {
            CARFAX.driveWithEncoder((int) Range.clip((-getGamepad1().left_stick_y + getGamepad1().left_stick_x) * 100, -100, 100), (int) Range.clip((-getGamepad1().left_stick_y - getGamepad1().left_stick_x) * 100, -100, 100));
        }else if(getGamepad2Functions().hasF1()) {
            CARFAX.driveWithEncoder((int) Range.clip((-getGamepad2().left_stick_y + getGamepad2().left_stick_x)  * 100, -100, 100), (int) Range.clip((-getGamepad2().left_stick_y - getGamepad2().left_stick_x) * 100, -100, 100));
        }
        if(getGamepad1Functions().hasF2()) {
            SPINNER.driveWithEncoder((int) Range.clip((getGamepad1().left_trigger - getGamepad1().right_trigger) * 100, -100, 100));
            if(getGamepad1().right_bumper && !getGamepad1().left_bumper) {
                LIFT.driveWithEncoder(50);
            }else if(!getGamepad1().right_bumper && getGamepad1().left_bumper){
                LIFT.driveWithEncoder(-50);
            }else{
                LIFT.driveWithEncoder(0);
            }
        }else if(getGamepad2Functions().hasF2()) {
            SPINNER.driveWithEncoder((int) Range.clip((getGamepad1().left_trigger - getGamepad1().right_trigger) * 100, -100, 100));
            if(getGamepad1().right_bumper && !getGamepad1().left_bumper) {
                LIFT.driveWithEncoder(50);
            }else if(!getGamepad1().right_bumper && getGamepad1().left_bumper){
                LIFT.driveWithEncoder(-50);
            }else{
                LIFT.driveWithEncoder(0);
            }
        }
    }

    @Override
    public void stop() {

    }

}
