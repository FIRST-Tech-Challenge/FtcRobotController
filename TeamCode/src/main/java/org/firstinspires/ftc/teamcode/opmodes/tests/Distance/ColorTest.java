package org.firstinspires.ftc.teamcode.opmodes.tests.Distance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;

@Autonomous
public class ColorTest extends LinearOpMode {
    // Define a variable for our color sensor
    TurtleRobotAuto robot = new TurtleRobotAuto(this);

    ColorSensor color;
    DistanceSensor distance;

    private double ColorDistance;
    private Double StraightDistance;
    static final double FEET_PER_METER = 3.28084;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   =  3.7795276;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");
        distance = hardwareMap.get(DistanceSensor.class, "Distance");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {

            telemetry.addData("ColorDistance", color.getDistance(DistanceUnit.INCH));
            telemetry.addData("", "");
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("","");

            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();

        }

    }

    public void Distance(double Distance_from_Robot, double Alignment) {
        distance.getDistance(DistanceUnit.INCH);



    public void Allignment() {
        ColorDistance = color.getDistance(DistanceUnit.CM);
        StraightDistance = distance.getDistance(DistanceUnit.CM);
        EncoderDrive(robot, 0.3, 7, StraightDistance - 7, StraightDistance - 7, StraightDistance - 7, 4);
        if (color.red() > color.green() && color.red() > color.blue() || (color.blue() > color.green() && color.blue() > color.blue())) {

        }

    }
        public void EncoderDrive(TurtleRobotAuto turtleRobotAuto, double speed,
        double leftfrontInches, double leftbackInches,
        double rightfrontInches, double rightbackInches,
        double timeoutS) {
            int newLeftfrontTarget;
            int newLeftbackTarget;
            int newRightfrontTarget;
            int newRightbackTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftfrontTarget = robot.leftfrontmotor.getCurrentPosition() + (int) (leftfrontInches * COUNTS_PER_INCH);
                newLeftbackTarget = robot.leftbackmotor.getCurrentPosition() + (int) (leftbackInches * COUNTS_PER_INCH);
                newRightfrontTarget = robot.rightfrontmotor.getCurrentPosition() + (int) (rightfrontInches * COUNTS_PER_INCH);
                newRightbackTarget = robot.rightbackmotor.getCurrentPosition() + (int) (rightbackInches * COUNTS_PER_INCH);
                robot.leftfrontmotor.setTargetPosition(newLeftfrontTarget);
                robot.leftbackmotor.setTargetPosition(newLeftfrontTarget);
                robot.rightfrontmotor.setTargetPosition(newRightfrontTarget);
                robot.rightbackmotor.setTargetPosition(newRightbackTarget);


                // Turn On RUN_TO_POSITION
                robot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftbackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightbackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftfrontmotor.setPower(Math.abs(speed));
                robot.leftbackmotor.setPower(Math.abs(speed));
                robot.rightfrontmotor.setPower(Math.abs(speed));
                robot.rightbackmotor.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the  will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the  continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS)
                        && (robot.leftfrontmotor.isBusy() &&
                        robot.leftbackmotor.isBusy()
                        && robot.rightfrontmotor.isBusy()
                        && robot.rightbackmotor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d",
                            newLeftfrontTarget,
                            newLeftbackTarget,
                            newRightfrontTarget,
                            newRightbackTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftfrontmotor.getCurrentPosition(),
                            robot.leftbackmotor.getCurrentPosition(),
                            robot.rightfrontmotor.getCurrentPosition(),
                            robot.rightbackmotor.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                robot.leftfrontmotor.setPower(0);
                robot.leftbackmotor.setPower(0);
                robot.rightfrontmotor.setPower(0);
                robot.rightbackmotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        }
    }
