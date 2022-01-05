package org.firstinspires.ftc.teamcode.main.utils.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.main.utils.autonomous.location.pipeline.PositionSystem;
import org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.NavigationSensorCollection;
import org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance.wrappers.UltrasonicDistanceSensor;
import org.firstinspires.ftc.teamcode.main.utils.interactions.groups.StandardDrivetrain;
import org.firstinspires.ftc.teamcode.main.utils.interactions.groups.StandardTankVehicleDrivetrain;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardDistanceSensor;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardIMU;
import org.firstinspires.ftc.teamcode.main.utils.io.InputSpace;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

@TeleOp(name = "Pipeline Test")
public class PipelineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NavigationSensorCollection sensors = new NavigationSensorCollection(
                new StandardDistanceSensor(hardwareMap, Resources.Navigation.Sensors.Distance.North),
                new StandardDistanceSensor(hardwareMap, Resources.Navigation.Sensors.Distance.East),
                new StandardDistanceSensor(hardwareMap, Resources.Navigation.Sensors.Distance.West),
                new StandardIMU(hardwareMap),
                90
        );

        PositionSystem positionSystem = new PositionSystem(sensors);
/*        InputSpace input = new InputSpace(hardwareMap);
        StandardTankVehicleDrivetrain tank = (StandardTankVehicleDrivetrain) input.getTank().getInternalInteractionSurface();
        positionSystem.setDrivetrain(tank);*/

        waitForStart();

        while (opModeIsActive()) {
            positionSystem.getAndEvalReadings();
            telemetry.addData("Angle (Degrees)", positionSystem.coordinateSystem.angle.toString());
            telemetry.addData("Position", positionSystem.coordinateSystem.current.toString());
            telemetry.update();
        }
    }
}
