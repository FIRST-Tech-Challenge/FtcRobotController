package org.firstinspires.ftc.teamcode.Mechanisms.Lift.Tuners;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.makePoseVector;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Pivot.Pivot;

@Config
@Autonomous(name = "Test Lift Feed Forward", group = "Autonomous")
public class TuneFeedForwardLift  extends LinearOpMode {
//    Encoder encoder;
    Lift lift;
    Arm arm;
    Claw claw;
    Extension extension;
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
        extension = new Extension(hardwareMap);
        pivot = new Pivot(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, battery);
        ElapsedTime looptime = new ElapsedTime();
        telemetry.update();
        waitForStart();

                Actions.runBlocking(
                        new SequentialAction(
                        drivetrain.goToPose(makePoseVector(10, 20, 0)),
                        extension.servoExtension(Extension.extensionState.RETRACT),
                        pivot.flippyFlip(Intake.intakeState.STOP),
                        new SleepAction(1),
                        arm.armClose(),
                        new SleepAction(1),
                        claw.servoClaw(Claw.clawState.OPEN),
                        new SleepAction(1),
                        lift.moveToHeight(28),
                        new SleepAction(1),
                        arm.armOpen(),
                        new SleepAction(1),
                        claw.servoClaw(Claw.clawState.CLOSE),
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
