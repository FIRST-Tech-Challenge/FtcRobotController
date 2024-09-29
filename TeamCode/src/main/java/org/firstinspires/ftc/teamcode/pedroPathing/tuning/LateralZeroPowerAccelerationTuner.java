package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@Autonomous (name = "Lateral Zero Power Acceleration Tuner", group = "Autonomous Pathing Tuning")
public class LateralZeroPowerAccelerationTuner extends OpMode {
    private ArrayList<Double> accelerations = new ArrayList<>();

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private PoseUpdater poseUpdater;

    public static double VELOCITY = 30;

    private double previousVelocity;

    private long previousTimeNano;

    private Telemetry telemetryA;

    private boolean stopping;
    private boolean end;

    /**
     * This initializes the drive motors as well as the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        poseUpdater = new PoseUpdater(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // TODO: Make sure that this is the direction your motors need to be reversed in.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("The robot will run to the right until it reaches " + VELOCITY + " inches per second.");
        telemetryA.addLine("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetryA.addLine("Make sure you have enough room.");
        telemetryA.addLine("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.");
        telemetryA.addLine("Press CROSS or A on game pad 1 to stop.");
        telemetryA.update();
    }

    /**
     * This starts the OpMode by setting the drive motors to run forward at full power.
     */
    @Override
    public void start() {
        leftFront.setPower(1);
        leftRear.setPower(-1);
        rightFront.setPower(-1);
        rightRear.setPower(1);
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing CROSS or A on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    @Override
    public void loop() {
        if (gamepad1.cross || gamepad1.a) {
            requestOpModeStop();
        }

        poseUpdater.update();
        Vector heading = new Vector(1.0, poseUpdater.getPose().getHeading() - Math.PI/2);
        if (!end) {
            if (!stopping) {
                if (MathFunctions.dotProduct(poseUpdater.getVelocity(), heading) > VELOCITY) {
                    previousVelocity = MathFunctions.dotProduct(poseUpdater.getVelocity(), heading);
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    for (DcMotorEx motor : motors) {
                        motor.setPower(0);
                    }
                }
            } else {
                double currentVelocity = MathFunctions.dotProduct(poseUpdater.getVelocity(), heading);
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < FollowerConstants.pathEndVelocityConstraint) {
                    end = true;
                }
            }
        } else {
            double average = 0;
            for (Double acceleration : accelerations) {
                average += acceleration;
            }
            average /= (double)accelerations.size();

            telemetryA.addData("lateral zero power acceleration (deceleration):", average);
            telemetryA.update();
        }
    }
}
