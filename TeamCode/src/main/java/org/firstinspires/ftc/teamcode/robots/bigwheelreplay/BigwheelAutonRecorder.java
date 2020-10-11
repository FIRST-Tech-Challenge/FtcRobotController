package org.firstinspires.ftc.teamcode.robots.bigwheelreplay;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;

@TeleOp(name="Big Wheel Auton Recorder", group="Linear Opmode")
public class BigwheelAutonRecorder extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor rightDrive2 = null;
    List<GamepadPair> log;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive1  = hardwareMap.get(DcMotor.class, "left_drive_1");
        leftDrive2  = hardwareMap.get(DcMotor.class, "left_drive_2");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive_1");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive_2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive1.setDirection(REVERSE);
        leftDrive2.setDirection(REVERSE);
        rightDrive1.setDirection(FORWARD);
        rightDrive2.setDirection(FORWARD);

        log = new ArrayList<>();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        for (int i = 0; opModeIsActive(); i++) {
            telemetry.addData("Iteration", "%d", i);
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;
            GamepadPair pair = new GamepadPair(gamepad1.left_stick_x, gamepad1.left_stick_y);
            log.add(pair);
            runIteration(pair);
            sleep(100);
        }

        String output = format(log);

        telemetry.addData("BigwheelAutonLog", output);
        log:logLargeString("BigwheelAutonLog", output);
    }

    public static class GamepadPair {
        public double x, y;
        public GamepadPair() {}
        public GamepadPair(double x, double y) {
            this.x=x; this.y=y;
        }
        @Override
        public String toString() {
            return String.format("(X: %.2f, Y: %.2f)", x, y);
        }
    }

    public void runIteration(GamepadPair pair) {
        double drive = -pair.y;
        double turn  =  pair.x;

        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        leftDrive1.setPower(leftPower);
        leftDrive2.setPower(leftPower);
        rightDrive1.setPower(rightPower);
        rightDrive2.setPower(rightPower);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
    }

    private static String format(List<GamepadPair> log) {
        StringBuilder str = new StringBuilder(
                            "package org.firstinspires.ftc.teamcode;\n\n" +

                            "import org.firstinspires.ftc.teamcode.robots.bigwheelreplay.BigwheelAutonRecorder.GamepadPair;\n\n" +

                            "public final class BigwheelAutonData {\n\n" +

                                "\tprivate BigwheelAutonData() {throw new RuntimeException();}\n\n" +

                                "\tpublic static java.util.List<GamepadPair> log = new java.util.ArrayList(").append(log.size()).append(") {{\n");

        for (GamepadPair pair : log) {
            str.append(             "\t\t\tadd(new GamepadPair(").append(pair.x).append(",").append(pair.y).append("));\n");
        }

        return str.append(      "\t}};\n" +
                            "}\n")
                   .toString();
    }

    public void logLargeString(String TAG, String str) {
        if(str.length() > 3000) {
            Log.i(TAG, str.substring(0, 3000));
            logLargeString(TAG, str.substring(3000));
        } else {
            Log.i(TAG, str); // continuation
        }
    }

}
