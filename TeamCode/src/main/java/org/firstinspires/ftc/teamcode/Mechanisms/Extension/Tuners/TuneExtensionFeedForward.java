package org.firstinspires.ftc.teamcode.Mechanisms.Extension.Tuners;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;

@Config
@Autonomous(name = "Tune Extension Feed Forward", group = "Autonomous")
public class TuneExtensionFeedForward  extends LinearOpMode {
    //    Encoder encoder;
    Extension extension;
    public static double height = 24;
    public static double height2 = 12;
    FtcDashboard dashboard;
    Drivetrain drivetrain;
    public void runOpMode() throws InterruptedException {
        Battery battery = new Battery(hardwareMap);
        Extension extension = new Extension(hardwareMap, battery);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        drivetrain = new Drivetrain(hardwareMap, battery);
        ElapsedTime looptime = new ElapsedTime();
        telemetry.update();
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        extension.moveToLength(height),
                        new SleepAction(1),
                        extension.moveToLength(height2),
                        new SleepAction(1),
                        extension.infiniteHold()
                        //lift.moveToHeight(24),
                        //new SleepAction(1),
                        //lift.moveToHeight(12)
                )
        );
//                if (Timer.seconds() > 3) {
//                    Actions.runBlocking(lift.moveToHeight(currentPosition));
//
//                    new Action() {
//                        @Override
//                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                            telemetryPacket.addLine("Height: " + lift.currentPosition);
//                                    telemetryPacket.addLine("Velocity: " + lift.liftMotorLeft.getVelocity());
//                                    telemetryPacket.addLine("Acceleration: " + lift.liftMotorLe);
//                                    telemetryPacket.addLine("maxHeight: " + maxHeight);
//                            return false;
//                        }
//                    };
    }
}
