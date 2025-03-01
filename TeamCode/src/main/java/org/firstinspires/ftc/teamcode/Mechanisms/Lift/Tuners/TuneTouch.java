package org.firstinspires.ftc.teamcode.Mechanisms.Lift.Tuners;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;

@Config
@Autonomous(name = "Tune Touch Sensor", group = "Autonomous")
public class TuneTouch extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
//    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {
        Battery battery = new Battery(hardwareMap);
        Lift lift = new Lift(hardwareMap, battery);
        waitForStart();
        while (opModeIsActive()) {
//            for (Action action : runningActions) {
//                action.preview(packet.fieldOverlay());
//                if (action.run(packet)) {
//                    newActions.add(lift.manualControl());
//                }
//            }
            //Actions.runBlocking(lift.manualControl(gamepad1.left_stick_y));
            TelemetryPacket packet = new TelemetryPacket();
            lift.manualControl(gamepad1.left_stick_y).run(packet);
            if (lift.limiter.isPressed()){
                packet.put("touch sensor", "pressed");
            } else {
                packet.put("touch sensor", "not pressed");
            }
            dash.sendTelemetryPacket(packet);
        }
    }

}
