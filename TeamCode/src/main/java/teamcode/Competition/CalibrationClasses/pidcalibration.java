package teamcode.Competition.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import teamcode.common.AbstractOpMode;
@Disabled
@TeleOp(name="pidcalibrate")
public class pidcalibration extends AbstractOpMode {

    PIDController pidShoot;
    DcMotor flywheel;
    @Override
    protected void onInitialize() {
        PIDCoefficients coefficients = new PIDCoefficients(1,0.05,1);
        flywheel = hardwareMap.dcMotor.get("Shooter");
        pidShoot = new PIDController(coefficients.p,coefficients.i, coefficients.d);

    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            double flywheelPower = 0.85;


        }
    }


    @Override
    protected void onStop() {

    }
}
