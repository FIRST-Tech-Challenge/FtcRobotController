package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.revextension2.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.revextension2.ExpansionHubMotor;
import org.firstinspires.ftc.teamcode.revextension2.ExpansionHubServo;
import org.firstinspires.ftc.teamcode.team.control.StackTracker;
import org.firstinspires.ftc.teamcode.team.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team.subsystems.ExpansionHubs;
import org.firstinspires.ftc.teamcode.team.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.team.subsystems.RobotStateEstimator;

import java.util.Arrays;

/**
 * Motor naming convention:
 *     Drivetrain
 *         Front Left Wheel -> LF
 *         Back Left Wheel   -> LR
 *         Front Right Wheel -> RF
 *         Back Right Wheel  -> RR
 *      Elevator
 *         Left Motor -> Elev Left
 *         Right Motor -> Elev Right
 *     Arm
 *         Arm Servo  -> Arm
 *      Claw
 *          Gripper -> Claw
 * Misc. sensors naming convention:

 */
public abstract class PPRobot extends Robot {
    //private  RevBlinkinLedDriver lights;
    private TimeProfiler matchRuntime;
    private ExpansionHubs expansionHubs;
    private RobotStateEstimator robotStateEstimator;
    private Drive drive;
    //Yogesh 12/28/2022 why does this class have liftSubsystem... commented...
    //private LiftSubsystem liftSubsystem;
    private Feeder feeder;
    private StackTracker stackTracker;


    @Override
    public void init() {
        super.init();
        setExpansionHubs(new ExpansionHubs(this,
                hardwareMap.get(ExpansionHubEx.class, "Control Hub"),
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"))
        );

        setMotors(new RevMotor[] {
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("LR")), true, false, true, false, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("LF")), true, false, true, false, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("RR")), true, false, true, true, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("RF")), true, false, true, true, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),

                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Elev Left")), false, true, false, true, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 0.7402879093),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Elev Right")), false, true, false, true, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 0.7402879093)});

        setServos(new RevServo[] {
                new RevServo((ExpansionHubServo)(hardwareMap.get("Arm"))),
                new RevServo((ExpansionHubServo)(hardwareMap.get("Claw")))
        });

        setRobotStateEstimator(new RobotStateEstimator(this, hardwareMap.get(BNO055IMU.class, "imu"), new Pose2d()));
        setDrive(new Drive(getRobotStateEstimator(), getMotors()[0], getMotors()[1], getMotors()[2], getMotors()[3]));
        //setLiftSubsystem(new LiftSubsystem(getMotors()[4], getMotors()[5]));
        setStackTracker(new StackTracker());
        setFeeder(new Feeder(getStackTracker(),getMotors()[4], getMotors()[5], getServos()[1], getServos()[0], hardwareMap.get(Rev2mDistanceSensor.class, "sensor_range")));
        setMatchRuntime(new TimeProfiler(false));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        getExpansionHubs().start();
        getDrive().start();
        Arrays.stream(getMotors()).forEach(RevMotor::resetEncoder);
      //  getLiftSubsystem().start();
        getFeeder().start();
        getMatchRuntime().start();
    }

    @Override
    public void loop() {
        super.loop();
        getExpansionHubs().update(getDt());
        getDrive().update(getDt());
        //getLiftSubsystem().update(getDt());
        getFeeder().update(getDt());
    }

    @Override
    public void stop() {
        super.stop();
        getExpansionHubs().stop();
        getDrive().stop();
        //getLiftSubsystem().stop();
        getFeeder().stop();
    }

    public ExpansionHubs getExpansionHubs() {
        return expansionHubs;
    }

    public void setExpansionHubs(ExpansionHubs expansionHubs) {
        this.expansionHubs = expansionHubs;
    }

    public RobotStateEstimator getRobotStateEstimator() {
        return robotStateEstimator;
    }

    public void setRobotStateEstimator(RobotStateEstimator robotStateEstimator) {
        this.robotStateEstimator = robotStateEstimator;
    }

    public Drive getDrive() {
        return drive;
    }

    public void setDrive(Drive drive) {
        this.drive = drive;
    }

//    public LiftSubsystem getLiftSubsystem() {
//        return liftSubsystem;
//    }

//    public void setLiftSubsystem(LiftSubsystem liftSubsystem){
//        this.liftSubsystem = liftSubsystem;
//    }

    public Feeder getFeeder() {
        return feeder;
    }


    public void setFeeder(Feeder feeder){
        this.feeder = feeder;
    }
   public StackTracker getStackTracker() {
        return stackTracker;
    }

    public void setStackTracker(StackTracker stackTracker) {
        this.stackTracker = stackTracker;
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

    public Pose2d getRobotPose() {
        return getRobotStateEstimator().getPose();
    }

    public double getRobotSpeed() {
        return getRobotStateEstimator().getVelocityPose().getTranslation().norm() +
                Math.abs(getRobotStateEstimator().getVelocityPose().getRotation().getRadians());
    }
}
