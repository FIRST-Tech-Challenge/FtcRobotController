package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;

@Disabled
@Autonomous(name="wobbletest")
public class WGGTest extends AbstractOpMode {
    private DcMotor armMotor;
    private Servo claw;
    @Override
    protected void onInitialize() {
        claw = hardwareMap.servo.get("Claw");
        armMotor = hardwareMap.dcMotor.get("Arm");
        Debug.log(claw.getPortNumber());
        Debug.log(armMotor.getPortNumber());

    }

    @Override
    protected void onStart() {

        try {
            claw.setPosition(0.15);
            Debug.log(claw.getPosition());
            Thread.sleep(3000);
            claw.setPosition(0.9);
            Debug.log(claw.getPosition());
            Thread.sleep(1500);
            armMotor.setPower(0.5);
            Debug.log(armMotor.getPower());
            Thread.sleep(1500);
            armMotor.setPower(-0.5);
            Debug.log(armMotor.getPower());
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
