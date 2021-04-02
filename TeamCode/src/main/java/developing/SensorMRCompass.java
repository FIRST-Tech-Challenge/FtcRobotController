package developing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

@TeleOp(name = "Sensor: MR compass", group = "Sensor")
@Disabled   // comment out or remove this line to enable this opmode
public class SensorMRCompass extends LinearOpMode {

    ModernRoboticsI2cCompassSensor compass;
    ElapsedTime timer = new ElapsedTime();
    TestRobot bot = new TestRobot();

    @Override public void runOpMode() {

        // get a reference to our compass
        compass = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "cp");

        telemetry.log().setCapacity(20);
        telemetry.log().add("The compass sensor operates quite well out-of-the");
        telemetry.log().add("box, as shipped by the manufacturer. Precision can");
        telemetry.log().add("however be somewhat improved with calibration.");
        telemetry.log().add("");
        telemetry.log().add("To calibrate the compass once the opmode is");
        telemetry.log().add("started, make sure the compass is level, then");
        telemetry.log().add("press 'A' on the gamepad. Next, slowly rotate the ");
        telemetry.log().add("compass in a full 360 degree circle while keeping");
        telemetry.log().add("it level. When complete, press 'B'.");

        bot.init(hardwareMap);

        // wait for the start button to be pressed
        waitForStart();
        telemetry.log().clear();

        while (opModeIsActive()) {

            double forward = -gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double turn = -gamepad1.left_stick_x;


            bot.moveTeleOp(forward, strafe, turn);

            // If the A button is pressed, start calibration and wait for the A button to rise
            if (gamepad1.a && !compass.isCalibrating()) {

                telemetry.log().clear();
                telemetry.log().add("Calibration started");
                telemetry.log().add("Slowly rotate compass 360deg");
                telemetry.log().add("Press 'B' when complete");
                compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
                timer.reset();

                while (gamepad1.a && opModeIsActive()) {
                    doTelemetry();
                    idle();
                }
            }

            // If the B button is pressed, stop calibration and wait for the B button to rise
            if (gamepad1.b && compass.isCalibrating()) {

                telemetry.log().clear();
                telemetry.log().add("Calibration complete");
                compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);

                if (compass.calibrationFailed()) {
                    telemetry.log().add("Calibration failed");
                    compass.writeCommand(ModernRoboticsI2cCompassSensor.Command.NORMAL);
                }

                while (gamepad1.a && opModeIsActive()) {
                    doTelemetry();
                    idle();
                }
            }

            doTelemetry();
        }
    }

    protected void doTelemetry() {

        if (compass.isCalibrating()) {

            telemetry.addData("compass", "calibrating %s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");

        } else {

            // getDirection() returns a traditional compass heading in the range [0,360),
            // with values increasing in a CW direction
            telemetry.addData("heading", "%.1f", compass.getDirection());

            // getAcceleration() returns the current 3D acceleration experienced by
            // the sensor. This is used internally to the sensor to compute its tilt and thence
            // to correct the magnetometer reading to produce tilt-corrected values in getDirection()
            Acceleration accel = compass.getAcceleration();
            double accelMagnitude = Math.sqrt(accel.xAccel*accel.xAccel + accel.yAccel*accel.yAccel + accel.zAccel*accel.zAccel);
            telemetry.addData("accel", accel);
            telemetry.addData("accel magnitude", "%.3f", accelMagnitude);

            // getMagneticFlux returns the 3D magnetic field flux experienced by the sensor
            telemetry.addData("mag flux", compass.getMagneticFlux());
        }

        // the command register provides status data
        telemetry.addData("command", "%s", compass.readCommand());

        telemetry.update();
    }
}

