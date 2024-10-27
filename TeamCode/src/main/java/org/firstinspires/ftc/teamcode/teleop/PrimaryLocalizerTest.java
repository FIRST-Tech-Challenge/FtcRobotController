package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.PrimaryLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.SparkOdo;
import org.firstinspires.ftc.teamcode.subsystems.ThreeEncoderLocalizer;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.LocalizerInterface;

//@TeleOp(name="PrimaryLocalizerTest", group = "Subsystem Tests")
public class PrimaryLocalizerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Init Phase
        Mecanum robot = new Mecanum(hardwareMap);
        GamepadEvents controller1 = new GamepadEvents(gamepad1);
        ThreeEncoderLocalizer threeWheel = new ThreeEncoderLocalizer(hardwareMap);
        LocalizerInterface[] localizerArray = new LocalizerInterface[] {
                new SparkOdo(hardwareMap),
                threeWheel
        };
        PrimaryLocalizer localizer = new PrimaryLocalizer(localizerArray);

        telemetry.addLine("Awaiting Start");
        telemetry.update();
        waitForStart();

        //Run Phase
        while(!isStopRequested()){
            robot.drive(controller1.left_stick_y,controller1.left_stick_x,controller1.right_stick_x);

            telemetry.addData("Spark: ", localizer.getLocalizers()[0].toString());
            telemetry.addData("Deadwheels: ", localizer.getLocalizers()[1].toString());
            telemetry.addLine(localizer.toString());
            telemetry.addLine();
            telemetry.addLine(threeWheel.DebugLog());
            telemetry.update();
            controller1.update();
        }

    }
}
