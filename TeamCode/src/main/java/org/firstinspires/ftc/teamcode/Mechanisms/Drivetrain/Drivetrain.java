package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.inverseKinematics;
import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.l;
import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.r;
import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.w;

import androidx.annotation.NonNull;

import java.util.List;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers.DrivetrainMotorController;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers.GeometricController;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers.PoseController;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Geometry.Path;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Localizers.TwoWheelOdometery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Hardware.Actuators.DcMotorAdvanced;

@Config
public class Drivetrain {
    /**
     * Hardware map
     */
    HardwareMap hardwareMap = null;
    public SimpleMatrix state = new SimpleMatrix(6, 1);
    public TwoWheelOdometery twoWheelOdo;

    public DrivetrainMotorController motorController;
    public GeometricController geometricController;
    /**
     * Drive motors
     */
    public DcMotorAdvanced motorLeftFront = null;
    public DcMotorAdvanced motorLeftBack = null;
    public DcMotorAdvanced motorRightBack = null;
    public DcMotorAdvanced motorRightFront = null;
//    public DcMotorEx motorLeftFront = null;
//    public DcMotorEx motorLeftBack = null;
//    public DcMotorEx motorRightBack = null;
//    public DcMotorEx motorRightFront = null;
    public SimpleMatrix wheelPowerPrev = new SimpleMatrix(4, 1);
    public PoseController poseControl = new PoseController();
    public static double acceptablePowerDifference = 0.000001; // The acceptable difference between current and previous wheel power to make a hardware call
    public static double distanceThreshold = 1;
    public static double angleThreshold = 0.1;
    public static double maxVoltage = 12.5;
    SimpleMatrix initialState = new SimpleMatrix(6,1);
    public void setInitialPose(double x, double y, double theta){
        initialState.set(0,0,x);
        initialState.set(1,0,y);
        initialState.set(2,0,theta);
    }
    public Battery battery;

    public SimpleMatrix prevWheelSpeeds = new SimpleMatrix( new double[][]{
            new double[]{0},
            new double[]{0},
            new double[]{0},
            new double[]{0}
    });
    public SimpleMatrix stopMatrix = new SimpleMatrix( new double[][]{
            new double[]{0},
            new double[]{0},
            new double[]{0},
            new double[]{0}
    });
    public ElapsedTime deltaT = new ElapsedTime();
    public Drivetrain(HardwareMap hardwareMap, Battery battery){
        this.hardwareMap = hardwareMap;
        this.motorController = new DrivetrainMotorController(hardwareMap);
        this.geometricController = new GeometricController();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        this.motorLeftFront = new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, "lfm"), battery, maxVoltage);
        this.motorLeftBack = new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, "lbm"), battery, maxVoltage);
        this.motorRightBack = new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, "rbm"), battery, maxVoltage);
        this.motorRightFront = new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, "rfm"), battery, maxVoltage);

//        this.motorLeftFront = hardwareMap.get(DcMotorEx.class, "lfm");
//        this.motorLeftBack = hardwareMap.get(DcMotorEx.class, "lbm");
//        this.motorRightBack = hardwareMap.get(DcMotorEx.class, "rbm");
//        this.motorRightFront = hardwareMap.get(DcMotorEx.class, "rfm");

        this.motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        /**
         * Establish that motors will not be using their native encoders:
         * 'RUN_WITHOUT_ENCODER' does not actually run without encoders, it
         * deactivates the PID
         */
        this.motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /**
         * Establish zero-power behavior
         */
        this.motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**
         * Set zero power to all motors upon initialization
         */
        this.motorLeftFront.setPower(0);
        this.motorLeftBack.setPower(0);
        this.motorRightFront.setPower(0);
        this.motorRightBack.setPower(0);
        this.twoWheelOdo = new TwoWheelOdometery(hardwareMap);
        deltaT.reset();
    }
    public void localize() {
        state = initialState.plus(twoWheelOdo.calculate());
    }
    public void setPower(SimpleMatrix powers) {
        double u0 = powers.get(0, 0);
        double u1 = powers.get(1, 0);
        double u2 = powers.get(2, 0);
        double u3 = powers.get(3, 0);
        double u0Prev = wheelPowerPrev.get(0, 0);
        double u1Prev = wheelPowerPrev.get(1, 0);
        double u2Prev = wheelPowerPrev.get(2, 0);
        double u3Prev = wheelPowerPrev.get(3, 0);
        motorLeftFront.setPower(powers.get(0, 0));
        motorLeftBack.setPower(powers.get(1, 0));
        motorRightBack.setPower(powers.get(2, 0));
        motorRightFront.setPower(powers.get(3, 0));
        wheelPowerPrev.set(0,0,u0);
        wheelPowerPrev.set(1,0,u1);
        wheelPowerPrev.set(2,0,u2);
        wheelPowerPrev.set(3,0,u3);
    }
    public void setWheelSpeedAcceleration(SimpleMatrix wheelSpeeds, SimpleMatrix wheelAccelerations){
        setPower(motorController.calculate(wheelSpeeds,wheelAccelerations));
    }

    /*public void goToPose(SimpleMatrix desiredPose){
        SimpleMatrix pose = state.extractMatrix(0,3,0,1);
        SimpleMatrix wheelSpeeds = poseControl.calculate(pose, desiredPose);
        SimpleMatrix wheelAccelerations = wheelSpeeds.minus(prevWheelSpeeds).scale(1/deltaT.seconds());
        deltaT.reset();
        setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);
        prevWheelSpeeds = wheelSpeeds;
    }*/
    public Action goToPose(SimpleMatrix desiredPose) {
        return new Action() {
            //private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //if (!initialized) {
                //    initialized = true;
                //}
                localize();
                packet.put("Done", "no");
                SimpleMatrix pose = state.extractMatrix(0,3,0,1);
                SimpleMatrix wheelSpeeds = poseControl.calculate(pose, desiredPose);
//                SimpleMatrix wheelAccelerations = wheelSpeeds.minus(prevWheelSpeeds).scale(1/deltaT.seconds());
                SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);
                deltaT.reset();
                setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);
                prevWheelSpeeds = wheelSpeeds;
                packet.put("X", state.get(0,0));
                packet.put("Y", state.get(1,0));
                packet.put("Theta", Math.toDegrees(state.get(2,0)));
                packet.put("X Velocity", state.get(3,0));
                packet.put("Y Velocity", state.get(4,0));
                packet.put("Theta Velocity", state.get(5,0));
                packet.put("PID X", wheelSpeeds.get(0,0));
                packet.put("PID Y", wheelSpeeds.get(1,0));
                packet.put("PID Theta", wheelSpeeds.get(2,0));
                if (Math.abs(Utils.calculateDistance(state.get(0,0),state.get(1,0),desiredPose.get(0,0),desiredPose.get(1,0)))<distanceThreshold&&Math.abs(Utils.angleWrap(state.get(2,0)-desiredPose.get(2,0)))<angleThreshold){
                    setPower(stopMatrix);
                    packet.put("Done", "done");
                }
                return !(Math.abs(Utils.calculateDistance(state.get(0,0),state.get(1,0),desiredPose.get(0,0),desiredPose.get(1,0)))<distanceThreshold&&Math.abs(Utils.angleWrap(state.get(2,0)-desiredPose.get(2,0)))<angleThreshold);
            }
        };
    }
    public Action goToPoseImprecise(SimpleMatrix desiredPose) {
        return new Action() {
            //private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //if (!initialized) {
                //    initialized = true;
                //}
                localize();
                packet.put("Done", "no");
                SimpleMatrix pose = state.extractMatrix(0,3,0,1);
                SimpleMatrix wheelSpeeds = poseControl.calculate(pose, desiredPose);
//                SimpleMatrix wheelAccelerations = wheelSpeeds.minus(prevWheelSpeeds).scale(1/deltaT.seconds());
                SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);
                deltaT.reset();
                setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);
                prevWheelSpeeds = wheelSpeeds;
                packet.put("X", state.get(0,0));
                packet.put("Y", state.get(1,0));
                packet.put("Theta", Math.toDegrees(state.get(2,0)));
                packet.put("X Velocity", state.get(3,0));
                packet.put("Y Velocity", state.get(4,0));
                packet.put("Theta Velocity", state.get(5,0));
                packet.put("PID X", wheelSpeeds.get(0,0));
                packet.put("PID Y", wheelSpeeds.get(1,0));
                packet.put("PID Theta", wheelSpeeds.get(2,0));
                if (Math.abs(Utils.calculateDistance(state.get(0,0),state.get(1,0),desiredPose.get(0,0),desiredPose.get(1,0)))<distanceThreshold*10&&Math.abs(Utils.angleWrap(state.get(2,0)-desiredPose.get(2,0)))<angleThreshold){
                    setPower(stopMatrix);
                    packet.put("Done", "done");
                }
                return !(Math.abs(Utils.calculateDistance(state.get(0,0),state.get(1,0),desiredPose.get(0,0),desiredPose.get(1,0)))<distanceThreshold*10&&Math.abs(Utils.angleWrap(state.get(2,0)-desiredPose.get(2,0)))<angleThreshold);
            }
        };
    }
    public InstantAction stopMotors() {
        return new InstantAction(()->setPower(stopMatrix));
    }
    public Action followPath(Path path) {
        return new Action() {
            // Maybe you'd be able to put an elapsedTimer here?? ****
            ElapsedTime elapsedTimer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //if (!initialized) {
                //    initialized = true;
                //}
                localize();


                // Check if the distance between the current robot position (so maybe grab the pose from the state FIRST!)
                // and the path's final waypoint is less than the geometricController's xy lookahead. If that is the case,
                // you'd want to do what?
                SimpleMatrix pose = state.extractMatrix(0,3,0,1);
                // Rename to desiredPose! It contains a heading too so furthestPoint is misleading!
                SimpleMatrix desiredPose = geometricController.calculate(state, path);
                SimpleMatrix wheelSpeeds = poseControl.calculate(pose, desiredPose);
                // not sure why you're using distanceThreshold here.
                // it would be something like path.getMotionProfile.getVelocity(t), where t is an elapsed timer.
                // ****This timer should started when you start following this path. Maybe you can put it above?
                // It shouldn't have to be reset as it should be specific to the action where you're following the path.
                double maxScale = path.getMotionProfile().getVelocity(elapsedTimer.seconds())/wheelSpeeds.elementMaxAbs(); // You also need to grab the maximum of the ABSOLUTE VALUE of all the wheel speeeds..
                wheelSpeeds.scale(maxScale);
                SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);
                deltaT.reset(); // don't need this right?
                setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);
                prevWheelSpeeds = wheelSpeeds;

                // Please please please, write a function in drivetrain called isClose or something, which does this checking for you!!!
                // replace it with that in both the goToPose and the followPath
                if (!(Math.abs(Utils.calculateDistance(state.get(0,0),state.get(1,0),wheelSpeeds.get(0,0),wheelSpeeds.get(1,0)))<distanceThreshold)){
                    if (Math.abs(Utils.angleWrap(state.get(2,0)-wheelSpeeds.get(2,0)))<angleThreshold) {
                        setPower(stopMatrix);
                    }
                }
                return Math.abs(Utils.calculateDistance(state.get(0,0),state.get(1,0),wheelSpeeds.get(0,0),wheelSpeeds.get(1,0)))>distanceThreshold&&Math.abs(Utils.angleWrap(state.get(2,0)-wheelSpeeds.get(2,0)))>angleThreshold;
            }
        };
    }
    public Action updateTelemetry() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                localize();
                packet.put("x", state.get(0,0));
                packet.put("y", state.get(1,0));
                packet.put("theta", state.get(2,0));
                packet.put("uLf", motorController.uLf);
                packet.put("uLb", motorController.uLb);
                packet.put("uRb", motorController.uRb);
                packet.put("uRf", motorController.uRf);

                return false;
            }
        };
    }
    public Action manualControl(double ly, double lx, double rX){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double y = ly;
                double x = -lx;
                double rx = -rX;

//                        if(gamepad2.right_stick_y > 0){
//                            runningActions.add(arm.servoArm());
//                        }
                SimpleMatrix compensatedTwist = new SimpleMatrix(
                        new double[][]{
                                new double[]{r * x},
                                new double[] {r * y},
                                new double[]{(r / (l + w)) * rx},
                        }
                );
                double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1.0);
                setPower(inverseKinematics(compensatedTwist).scale(1/denominator));
                telemetryPacket.put("X", state.get(0,0));
                telemetryPacket.put("Y", state.get(1,0));
                telemetryPacket.put("Theta", Math.toDegrees(state.get(2,0)));
                return false;
            }
        };
    }
}
