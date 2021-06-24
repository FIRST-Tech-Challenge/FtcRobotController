package teamcode.Competition.CalibrationClasses;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Point;
import teamcode.common.Vector2D;

@Autonomous(name="pathing test")
public class DriveCalibration extends AbstractOpMode {

    MecanumDriveTrain driveTrain;
    Localizer localizer;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0);
        driveTrain = new MecanumDriveTrain(hardwareMap);
    }

    @Override
    protected void onStart() {
        //driveTrain.setPower(0.5,0.5,-0.5,-0.5);
       // driveTrain.forward(10, 0.1);
//        driveTrain.forward(-10, 0.5);
//        driveTrain.strafe(10,0.5);
//        driveTrain.strafe(-10, 0.5);
//        driveTrain.rotate(Math.PI / 2, 0.5, true);
//        driveTrain.rotate(0, 0.5, true);
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }
}
