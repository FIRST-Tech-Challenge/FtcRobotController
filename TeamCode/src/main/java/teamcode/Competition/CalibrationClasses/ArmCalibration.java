package teamcode.Competition.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.Competition.WobbleArm;
import teamcode.common.AbstractOpMode;


@Autonomous(name="arm")
public class ArmCalibration extends AbstractOpMode {
    WobbleArm arm;

    @Override
    protected void onInitialize() {
        arm = new WobbleArm(hardwareMap, true);
    }

    @Override
    protected void onStart() {
        //arm.runToPosition();
        while(opModeIsActive()){
            if(gamepad1.dpad_right){
                arm.setPower(0.5);
            }else if(gamepad1.dpad_left){
                arm.setPower(-0.5);
            }else{
                arm.setPower(0);
            }
            telemetry.addData("current pos", arm.getCurrentPos());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}
