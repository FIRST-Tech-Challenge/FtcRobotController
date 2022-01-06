package org.firstinspires.ftc.teamcode.main.utils.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.main.utils.autonomous.location.pipeline.PositionSystem;
import org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.NavigationSensorCollection;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardDistanceSensor;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardIMU;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

@TeleOp(name = "Pipeline Test")
public class PipelineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // FIXME: this method doesnt exist? not sure if its uncommitted or something but commenting out for now
//        PositionSystem positionSystem = Resources.Navigation.Sensors.getPositionSystem(hardwareMap);

/*        InputSpace input = new InputSpace(hardwareMap);
        StandardTankVehicleDrivetrain tank = (StandardTankVehicleDrivetrain) input.getTank().getInternalInteractionSurface();
        positionSystem.setDrivetrain(tank);*/

        waitForStart();
//
//        while (opModeIsActive()) {
//            positionSystem.getAndEvalReadings();
//            telemetry.addData("Angle (Degrees)", positionSystem.coordinateSystem.angle.toString());
//            telemetry.addData("Position", positionSystem.coordinateSystem.current.toString());
//            telemetry.update();
//        }
    }
}
