
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Mr. Price's teleOp for test and explanation - this one uses a Hardware Class structure
 *
 * Need to confirm the drive system and add the drone servo to have a full test case
 * Also need the telemetry to read all sensor values
 */

@TeleOp(name="coachMap", group="TeleOp")

public class coachMapped extends LinearOpMode {

    //vvHardware class external pull
    vvHardware   robot       = new vvHardware(this);

    public ColorSensor colorSensor;
    public DistanceSensor distFront;
    public DistanceSensor distRear;
@Override
    public void runOpMode() throws InterruptedException {
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        double RWPower = 0;
        double LWPower = 0;
        double drivePower = 0.5; //global drive power level
        double armPower = 0;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;
        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "CLR");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        distFront = hardwareMap.get(DistanceSensor.class, "FDS");
        distRear = hardwareMap.get(DistanceSensor.class, "RDS");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distFront;

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (isStopRequested()) return;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            while (opModeIsActive()) {
                driveY = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x * 1.1;
                turn = gamepad1.right_stick_x;

                armPower = gamepad2.left_stick_y;

                // Combine drive and turn for blended motion. Use RobotHardware class
                robot.driveRobot(0.5, driveY, strafe, turn);

                robot.moveArm(armPower);

                // Controlling the pixel pick-up with the dpad and buttons (individual)
                if (gamepad2.dpad_left) {
                    LWPower = 0.2;
                    RWPower = -0.2;
                } else if (gamepad2.dpad_right) {
                    LWPower = -0.2;
                    RWPower = 0.2;
                } else if (gamepad2.y)
                    LWPower = -0.2;
                else if (gamepad2.x)
                    LWPower = 0.2;
                else {
                    LWPower = 0;
                    RWPower = 0;
                }

// Adding telemetry readouts
                telemetry.addData(">", "Robot Running");
                telemetry.addData("Y", driveY);
                telemetry.addData("strafe", strafe);
                telemetry.addData("turn", turn);
// check the status of the x button on either gamepad.
                bCurrState = gamepad1.x;

                // check for button state transitions.
                if (bCurrState && (bCurrState != bPrevState)) {

                    // button is transitioning to a pressed state. So Toggle LED
                    bLedOn = !bLedOn;
                    colorSensor.enableLed(bLedOn);
                }

                // update previous state variable.
                bPrevState = bCurrState;

                // convert the RGB values to HSV values.
                Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

                // send the info back to driver station using telemetry function.
                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.addData("Hue", hsvValues[0]);

                telemetry.addData("FDS", distFront.getDeviceName());
                telemetry.addData("range", String.format("%.01f cm", distFront.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f in", distFront.getDistance(DistanceUnit.INCH)));

                telemetry.addData("RDS", distRear.getDeviceName());
                telemetry.addData("range", String.format("%.01f cm", distRear.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f in", distRear.getDistance(DistanceUnit.INCH)));

                // Rev2mDistanceSensor specific methods.
                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

                telemetry.update();

                // Use gamepad buttons to move arm up (Y) and down (A)
            /*if (gamepad1.y)
                leftArm.setPower(ARM_UP_POWER);
            else if (gamepad1.a)
                leftArm.setPower(ARM_DOWN_POWER);
            else
                leftArm.setPower(0.0);
            */
                // Send telemetry message to signify robot running;
               /* telemetry.addData("claw", "Offset = %.2f", clawOffset);
                telemetry.addData("left", "%.2f", left);
                telemetry.addData("right", "%.2f", right);
                telemetry.update();

                // Pace this loop so jaw action is reasonable speed.
                sleep(50);
                */

            }
        }
    }
}
