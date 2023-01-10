
package Team.ComplexRobots;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;


import Team.BasicRobots.BasicMecanum;

public class Robot extends BasicMecanum {

    public DcMotor linearSlidesMotor1;

    public void init(HardwareMap Map) {

        super.init(Map);

        linearSlidesMotor1 = Map.dcMotor.get("linearSlidesMotor1");

        linearSlidesMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlidesMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setLinearSlidePosition(double power, int position){

        linearSlidesMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlidesMotor1.setTargetPosition(position);
        linearSlidesMotor1.setPower(power);

    }

}