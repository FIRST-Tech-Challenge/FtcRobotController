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
        //enums to store const info on robot config
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
        /* other enums
         *
        */
    }
    
    //making a class to store driveMotors instead of a hashmap
    abstract class Motors {
        //init motor takes motor reference and then does some in-place changes to it
        //not actually sure this works, but it should so i hope it works xdddd
        private void initMotor(DcMotor motor, Config.DriveMotors config) {
            //I have no clue why you need Objects.requireNonNull(Object), left in just in case though
            Objects.requireNonNull(motor).setDirection(config.getDirection());
            Objects.requireNonNull(motor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Objects.requireNonNull(motor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Objects.requireNonNull(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    class DriveMotors extends Motors {
        public DcMotor frontLeft, frontRight, rearLeft, rearRight;
        //take hardware map and then init all motors
        public DriveMotors(HardwareMap hardwareMap) {
            frontLeft = hardwareMap.get(DcMotor.class, Config.DriveMotors.FRONT_LEFT);
            frontRight= hardwareMap.get(DcMotor.class, Config.DriveMotors.FRONT_RIGHT);
            rearLeft  = hardwareMap.get(DcMotor.class, Config.DriveMotors.REAR_LEFT);
            rearRight = hardwareMap.get(DcMotor.class, Config.DriveMotors.REAR_RIGHT);
            //pass dcmotor reference and set to config
            initMotor(frontLeft, Config.DriveMotors.FRONT_LEFT);
            initMotor(frontRight,Config.DriveMotors.FRONT_RIGHT);
            initMotor(rearLeft,  Config.DriveMotors.REAR_LEFT);
            initMotor(rearRight, Config.DriveMotors.REAR_RIGHT);
        }
    }
/*
    class SlideMotors extends Motors {
        public DcMotors slide;
        public DriveMotors(HardwareMap hardwareMap) {
            slide = hardwareMap.get(DcMotor.class, Config.SlideMotors.whatever);
            initMotor(slide, Config.SlideMotors.whatever);
        }
    }
*/
    
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
        this.telemetry   = telemtry;
        this.elapsedTime = elapsedTime;
        positionManger   = new PositionManager(hardwareMap, telemtry);
        driveMotors      = new DriveMotors(hardwareMap);

        /*whatever goes here
         *
         */

        driveMotors.frontLeft = hardwareMap.get(DcMotor.class, Robot)
    }
}
