package teamcode.offSeason;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;

@TeleOp(name="Encoder Calibrator")
public class EncoderCalibrator extends AbstractOpMode {

    private static final double MOTOR_DEGREES_TO_TICKS = 1.0;
    private static final int TICK_ROTATIONAL_TOLERANCE = 50;
    //1x motor
    //1x Expansion Hub
    DcMotor motor;

    @Override
    protected void onInitialize() {
        motor = hardwareMap.dcMotor.get("Motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    protected void onStart() {
        rotate(360, 0.5);
    }

    private void rotate(double degrees, double power) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int)(degrees * MOTOR_DEGREES_TO_TICKS);
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Debug.log(motor.getCurrentPosition());
//        Debug.log(motor.getTargetPosition());

        while(Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) > TICK_ROTATIONAL_TOLERANCE) {
            Debug.log(Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()));
            motor.setPower(power);
        }
        motor.setPower(0);
    }

    @Override
    protected void onStop() {

    }
}
