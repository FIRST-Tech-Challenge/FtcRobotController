package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver;
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;
import org.firstinspires.ftc.teamcode.hardware.HardwareName;
import org.firstinspires.ftc.teamcode.hardware.Reversed;
import org.firstinspires.ftc.teamcode.hardware.ZeroPower;
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.hardware.PoseSet;
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RobotState;

public class GerthaHardwareMap extends HardwareMapper {

    public static final double POLLEN_UP = 0.15;
    public static final double POLLEN_ONLY = 0.53;
    public static final double POLLEN_EXTRACT = 0.57;
    public static final double DISTANCE_THRESHOLD = 4.0;
    public static final double POLLEN_GO = 0.58;
    public static final double POLLEN_STOP = 0.34;
    public static final double AUTO_SHOOT_SPEED = 1120.0;
    public static final double ANGLE_OF_SHOT = 1.188;
    public static final double DISTANCE_TO_TOP = (64.0 - 13.5) * 0.0254;

    @HardwareName("frontRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx frontRight;

    @HardwareName("backRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx backRight;

    @HardwareName("frontLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx frontLeft;

    @HardwareName("backLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx backLeft;

    @HardwareName("pollenShooter")
    @Reversed
    public DcMotorEx pollenShooter;

    @HardwareName("leftIntake")
    public DcMotor leftIntake;

    @HardwareName("rightIntake")
    @Reversed
    public DcMotor rightIntake;

    @HardwareName("pinpoint")
    public GoBildaPinpoint2Driver pinpoint;

    @HardwareName("rightRamp")
    public Servo rightRamp;

    @HardwareName("leftRamp")
    public Servo leftRamp;

    @HardwareName("pollenBallStop")
    public Servo pollenBallStop;

    @HardwareName("color")
    public RevColorSensorV3 color;

    @HardwareName("Control Hub")
    public VoltageSensor voltageSensor;

    @HardwareName("pollenTurretServo")
    public Servo pollenTurret;

    @HardwareName("Webcam 1")
    public WebcamName webcam1;

    public GerthaHardwareMap(HardwareMap map) {
        super(map);

        pinpoint.setOffsets(4.78,  -5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpoint2Driver.EncoderDirection.REVERSED, GoBildaPinpoint2Driver.EncoderDirection.REVERSED);
        pollenShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(760, 40, 20, 0));

    }


    public static double normalize(double angle) {
        double tempAngle = angle % (2 * PI);
        if (tempAngle > PI) {
            tempAngle -= (2 * PI);
        } else if (tempAngle <= -PI) {
            tempAngle += (2 * PI);
        }
        return tempAngle;
    }

    public static double angleDifference(double angle1, double angle2){
        double diff = normalize(angle1) - normalize(angle2);
        diff = abs(diff);
        if (diff > PI) {
            diff = (2 * PI) - diff;
        }
        return diff;
    }

    public static double distance(RobotState robotState, PoseSet poseSet) {
        if (!robotState.getRightHive()){
            return Math.hypot((poseSet.getLEFT_HIVE_AT().x  - robotState.getPpX()), (poseSet.getLEFT_HIVE_AT().y - robotState.getPpY()));
        }
        else {
            return Math.hypot((poseSet.getRIGHT_HIVE_AT().x  - robotState.getPpX()), (poseSet.getRIGHT_HIVE_AT().y - robotState.getPpY()));
        }
    }

    public void setIntakePower(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public void setPollenExtractor(double pos) {
        rightRamp.setPosition(pos);
        leftRamp.setPosition(1.05 - pos);
    }

    public void setPollenVelocity(double target){
        pollenShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenShooter.setVelocity(target);
    }

}
