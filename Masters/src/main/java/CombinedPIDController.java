import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.masters.ArmPIDController;
import org.firstinspires.ftc.masters.LiftPIDController;

public class CombinedPIDController {

    LiftPIDController liftPIDController;
    ArmPIDController armPIDController;

    int armTarget=0, liftTarget =0;

    public CombinedPIDController(DcMotorEx armMotor, DcMotorEx liftMotor, DcMotorEx fromLiftMotor, DcMotorEx leftLiftMotor){
        liftPIDController = new LiftPIDController(liftMotor, fromLiftMotor, leftLiftMotor);
        armPIDController = new ArmPIDController(armMotor);
    }


}
