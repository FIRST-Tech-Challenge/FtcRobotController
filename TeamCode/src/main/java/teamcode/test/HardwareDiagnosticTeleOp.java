package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Point;
import teamcode.common.Vector2D;


@TeleOp(name="HardwareDiagnosticTO")
public class HardwareDiagnosticTeleOp extends AbstractOpMode {
    //tests odo and drive

    MecanumDriveTrain driveTrain;
    Localizer localizer;
    @Override
    protected void onInitialize() {
        driveTrain = new MecanumDriveTrain(hardwareMap);
        localizer = new Localizer(hardwareMap, new Point(0,0), 0);

    }

    @Override
    protected void onStart() {
        new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    driveTrain.setPower(new Vector2D(gamepad1.left_stick_x * 0.5, gamepad1.left_stick_y * 0.5), gamepad1.right_stick_x);
                }
            }
        }.start();
        while(opModeIsActive()){
            //telemetry.addData("Position ", localizer.getCurrentPosition());
            //telemetry.addData("Global Rads: ", localizer.getGlobalRads());
            telemetry.update();
        }
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
