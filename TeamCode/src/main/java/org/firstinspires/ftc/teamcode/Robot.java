package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;


/** Stores the Robot's hardware and position.
 *  Also has a "desired state" for mechanism driving.
 */
public class Robot {
    //static class Config instead of RobotConfig, access via Robot.Config
    static class Config {
        //using enums instead of hashmap spam
        public enum DriveMotors {
            FRONT_LEFT  ("front_left", DcMotor.Direction.REVERSE),
            FRONT_RIGHT ("front_right",DcMotor.Direction.REVERSE),
            REAR_LEFT   ("rear_left",  DcMotor.Direction.REVERSE),
            REAR_RIGHT  ("rear_right", DcMotor.Direction.REVERSE);
            
            private String name;
            private DcMotor.Direction direction;
            private DriveMotor(final String name, final DcMotor.Direction direction) {
                this.name = name;
                this.direction = direction;
            }
            //accessors
            public String            getName()      { return name; }
            public DcMotor.Direction getDirection() { return direction; }
        }
    }
    
    //making a class to store driveMotors instead of a hashmap
    class DriveMotors {
        public DcMotor frontLeft, frontRight, rearLeft, rearRight;
        //take hardware map and then init all motors
        public DriveMotors(HardwareMap hardwareMap) {
            //pass dcmotor reference and set to desired configs with hardwaremap
            initMotor(frontLeft, Config.DriveMotors.FRONT_LEFT, hardwareMap);
            initMotor(frontRight,Config.DriveMotors.FRONT_RIGHT,hardwareMap);
            initMotor(rearLeft,  Config.DriveMotors.REAR_LEFT,  hardwareMap);
            initMotor(rearRight, Config.DriveMotors.REAR_RIGHT, hardwareMap);
        }
        //init motor takes motor reference and then does some in-place changes to it
        private void initMotor(DcMotor motor, Config.DriveMotors config, HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotor.class, config.getName());
            //I have no clue why you need Objects.requireNonNull(Object), left in just in case though
            Objects.requireNonNull(motor).setDirection(config.getDirection());
            Objects.requireNonNull(motor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Objects.requireNonNull(motor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Objects.requireNonNull(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    
    //enum spam

    /*
     * pretend there's a lot of enums here
     */

    //robot parts
    public DriveMotors driveMotors;
    
    public Telemetry telemetry;
    public ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public PositionManager positionManager;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime elapsedTime) {
        this.telemetry = telemtry;
        this.elapsedTime = elapsedTime;
        positionManger = new PositionManage(hardwareMap, telemtry);
        driveMotors = new DriveMotors(hardwareMap);

        /*whatever goes here
         *
         */

        driveMotors.frontLeft = hardwareMap.get(DcMotor.class, Robot)
    }
}
