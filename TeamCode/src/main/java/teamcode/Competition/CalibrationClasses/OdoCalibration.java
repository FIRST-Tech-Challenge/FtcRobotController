package teamcode.Competition.CalibrationClasses;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Point;
import teamcode.common.PurePursuit.CurvePoint;
import teamcode.common.PurePursuit.MovementVars;
import teamcode.common.PurePursuit.PurePursuitMovement;
import teamcode.common.Vector2D;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class OdoCalibration extends AbstractOpMode {
    Localizer localizer;
    MecanumDriveTrain drive;
    PurePursuitMovement movement;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0);
    }

    @Override
    protected void onStart() {
        goTo(new Point(100,0), 1.0, 0.0);
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }

    private void goTo(Point point, double drivePower, double turnPower) {
       // while(!robotIsNearPoint(new CurvePoint(point)) && opModeIsActive()) {
//            movement.goToPosition(point.x, point.y, drivePower, 0, turnPower);
//            drive.setPowerPurePursuit(new Vector2D(MovementVars.movementX, -MovementVars.movementY), MovementVars.movementTurn);
//            telemetry.addData("current pos:" ,localizer.getCurrentPosition());
//            telemetry.addData("current rot:", localizer.getGlobalRads());
//            telemetry.update();
//        }
//        drive.setPower(0,0,0,0);
    }

//    public boolean robotIsNearPoint(CurvePoint current) {
//        Point currentPoint = current.toPoint();
//        return sqrt(pow(currentPoint.x - localizer.getCurrentPosition().x, 2) + pow(currentPoint.y - localizer.getCurrentPosition().y, 2)) <= 0.5;
//    }
}
