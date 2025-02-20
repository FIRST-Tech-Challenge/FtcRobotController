package org.firstinspires.ftc.teamcode.Mechanisms.Lift.Tuners;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.ExtensionOuttake;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Pivot.Pivot;

@Config
@Autonomous(name = "Tune Lift Feed Forward", group = "Autonomous")
public class TuneFeedForwardLift  extends LinearOpMode {
//    Encoder encoder;
    Lift lift;
    Arm arm;
    Claw claw;
    ExtensionOuttake extension;
    Pivot pivot;
    public static double height = 24;
    public static double height2 = 12;
    FtcDashboard dashboard;
    Drivetrain drivetrain;
    public void runOpMode() throws InterruptedException {
        Battery battery = new Battery(hardwareMap);
        lift = new Lift(hardwareMap, battery);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        extension = new ExtensionOuttake(hardwareMap);
        pivot = new Pivot(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, battery);
        ElapsedTime looptime = new ElapsedTime();
        telemetry.update();
        waitForStart();

                Actions.runBlocking(
                        new SequentialAction(
                        lift.moveToHeight(height),
                        new SleepAction(1),
                        lift.moveToHeight(height2),
                        new SleepAction(1),
                        lift.infiniteHold()
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
