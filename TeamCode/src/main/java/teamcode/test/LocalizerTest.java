package teamcode.test;

import android.graphics.drawable.VectorDrawable;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Point;
import teamcode.common.PurePursuit.CurvePoint;
import teamcode.common.PurePursuit.MovementVars;
import teamcode.common.PurePursuit.PurePursuitMovement;
import teamcode.common.RobotPositionStateUpdater;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@TeleOp(name="Localizer Test")
public class LocalizerTest extends AbstractOpMode {

    Localizer localizer;
    MecanumDriveTrain driveTrain;
    PurePursuitMovement movement;
    Thread movementAssignment;
    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0),0);
        driveTrain = new MecanumDriveTrain(hardwareMap, localizer);
    }

    @Override
    protected void onStart() {
        //driveTrain.setPower(-0.5,.5, -0.5, 0.5);
        localizer.start();
        driveTrain.moveToPosition(new Vector2D(0,24), 6.0, 0);

        //driveTrain.moveToRotation( Math.PI/2.0, 0.5);
        while(opModeIsActive()) {
            RobotPositionStateUpdater.RobotPositionState state = localizer.getCurrentState();
            telemetry.addData("", state.toString());
            telemetry.update();
        }

    }

    /*
    driveTrain.setPowerPurePursuit(new Vector2D( gamepad1.left_stick_x , gamepad1.left_stick_y ),
                    gamepad1.right_stick_x);
            telemetry.addData("position", localizer.getCurrentPosition());
            telemetry.addData("rads", localizer.getGlobalRads());
            telemetry.update();
     */
    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}
