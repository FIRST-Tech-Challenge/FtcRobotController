package developing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestRobot {

    public DcMotor r1;
    public DcMotor l1;
    public DcMotor r2;
    public DcMotor l2;

    public void init(HardwareMap hwMap) {

        l1 = hwMap.get(DcMotor.class, "l1");
        l2 = hwMap.get(DcMotor.class, "l2");
        r1 = hwMap.get(DcMotor.class, "r1");
        r2 = hwMap.get(DcMotor.class, "r2");

        l1.setPower(0);
        l2.setPower(0);
        r1.setPower(0);
        r2.setPower(0);

        l1.setDirection(DcMotorSimple.Direction.FORWARD);
        l2.setDirection(DcMotorSimple.Direction.REVERSE);
        r1.setDirection(DcMotorSimple.Direction.REVERSE);
        r2.setDirection(DcMotorSimple.Direction.FORWARD);

        l1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(double f, double s, double t){
        l1.setPower(f+s-t);
        l2.setPower(-f+s+t);
        r1.setPower(f-s+t);
        r2.setPower(-f-s-t);
    }
}
