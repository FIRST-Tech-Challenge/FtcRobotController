package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Color beacon code from https://modernroboticsinc.com/product/color-beacon/

public class Magazine {
    //public ElapsedTime magazineTimer;
    public CRServo magazineServo;
    public NormalizedColorSensor magazineRingSensor;


    public I2cDevice magazineColorBeacon;
    public I2cDeviceSynch magazineColorBreader;

    public Servo magazineBeaconServo;

    public TouchSensor magazineLaunchTouchSensor;
    public TouchSensor magazineCollectTouchSensor;

    LinearOpMode opModepassed;

    public enum MAGAZINE_POSITION {
        AT_COLLECT,
        AT_LAUNCH,
        AT_ERROR
    }

    public MAGAZINE_POSITION magazinePosition = MAGAZINE_POSITION.AT_ERROR;
    public boolean magazinePositionError = false;

    public enum MAGAZINE_TOUCH_SENSORS_STATE {
        LAUNCH_TS_PRESSED,
        LAUNCH_TS_NOT_PRESSED,
        COLLECT_TS_PRESSED,
        COLLECT_TS_NOT_PRESSED,
        TS_ERROR
    }

    public enum MAGAZINE_RING_COUNT {
        ZERO,
        ONE,
        TWO,
        THREE
    }

    public MAGAZINE_RING_COUNT magazineRingCount = MAGAZINE_RING_COUNT.ZERO;

    //TODO : AMJAD : Better coding of enum with values at https://www.baeldung.com/java-enum-values
    public static final double RING_NONE_DISTANCE = 3.45;
    public static final double RING_ONE_DISTANCE = 3.0;
    public static final double RING_TWO_DISTANCE = 2.0;
    public static final double RING_THREE_DISTANCE = 0.5;

    public double magazine_distance;

    //TODO : AMJAD : Use servo to flag if beacon is not working
    public static final double magazineBeaconServoPos_MAGAZINE_RINGS_0 = 0.0;
    public static final double magazineBeaconServoPos_MAGAZINE_RINGS_1 = 0.25;
    public static final double magazineBeaconServoPos_MAGAZINE_RINGS_2 = 0.5;
    public static final double magazineBeaconServoPos_MAGAZINE_RINGS_3 = 1.0;

    public Magazine(HardwareMap hardwareMap) {
        magazineServo = hardwareMap.crservo.get("mgz_servo");
        magazineRingSensor = hardwareMap.get(NormalizedColorSensor.class, "mgz_ring_sensor");

        //magazineColorBeacon = hardwareMap.i2cDevice.get("mgz_beacon");
       // magazineColorBreader = new I2cDeviceSynchImpl(magazineColorBeacon, I2cAddr.create8bit(0x4c), false);
        //magazineColorBreader.engage();

        magazineBeaconServo = hardwareMap.servo.get("mgz_beacon_servo");

        magazineLaunchTouchSensor = hardwareMap.touchSensor.get("mgz_launch_ts");
        magazineCollectTouchSensor = hardwareMap.touchSensor.get("mgz_collect_ts");

    }

    public void initMagazine(LinearOpMode opModepassed){
         this.opModepassed = opModepassed;

        //magazineLaunchTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        //magazineCollectTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        senseMagazinePosition();
        senseMagazineRingStatus();
    }

    public void senseMagazinePosition(){
        MAGAZINE_TOUCH_SENSORS_STATE magazine_touch_sensors_state = getMagazineTouchSensorsState();
        if (magazine_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_PRESSED) {
            magazinePosition = MAGAZINE_POSITION.AT_LAUNCH;
            magazinePositionError = false;
        } else if (magazine_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_PRESSED) {
            magazinePosition = MAGAZINE_POSITION.AT_COLLECT;
            magazinePositionError = false;
        } else if (magazine_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.TS_ERROR){
            //Retain old magazinePosition State
            //magazinePosition = MAGAZINE_POSITION.AT_ERROR;
            magazinePositionError = true;
        }
    }

    public MAGAZINE_TOUCH_SENSORS_STATE getMagazineTouchSensorsState(){
        MAGAZINE_TOUCH_SENSORS_STATE magazine_touch_sensors_state;
        MAGAZINE_TOUCH_SENSORS_STATE magazine_launch_touch_sensors_state;
        MAGAZINE_TOUCH_SENSORS_STATE magazine_collect_touch_sensors_state;

        magazine_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.TS_ERROR;
        if (magazineLaunchTouchSensor.isPressed()){
            magazine_touch_sensors_state =  MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_PRESSED;
            magazine_launch_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_PRESSED;
        } else {
            magazine_launch_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_NOT_PRESSED;
        }

        if (magazineCollectTouchSensor.isPressed()){
            magazine_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_PRESSED;
            magazine_collect_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_PRESSED;
        } else {
            magazine_collect_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_NOT_PRESSED;
        }

        if ((magazine_launch_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_NOT_PRESSED &&
                magazine_collect_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_NOT_PRESSED) ||
                (magazine_launch_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_PRESSED &&
                        magazine_collect_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_PRESSED)){
            magazine_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.TS_ERROR;
        }

        return magazine_touch_sensors_state;
    }

    public void turnMagazineBeaconOff() {
        magazineBeaconServo.setPosition(magazineBeaconServoPos_MAGAZINE_RINGS_0);
        //magazineColorBreader.write8(4, 0);
    }

    public void turnMagazineBeaconPurple() {
        magazineBeaconServo.setPosition(magazineBeaconServoPos_MAGAZINE_RINGS_1);
        //magazineColorBreader.write8(4, 5);
    }

    public void turnMagazineBeaconTeal() {
        magazineBeaconServo.setPosition(magazineBeaconServoPos_MAGAZINE_RINGS_2);
        //magazineColorBreader.write8(4, 6);
    }

    public void turnMagazineBeaconWhite() {
        magazineBeaconServo.setPosition(magazineBeaconServoPos_MAGAZINE_RINGS_3);
        //magazineColorBreader.write8(4, 7);
    }

    public boolean moveMagazineToLaunch() {
        senseMagazinePosition();
        //senseMagazineRingStatus();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        if(/*magazinePosition != MAGAZINE_POSITION.AT_ERROR &&*/
                magazinePosition != MAGAZINE_POSITION.AT_LAUNCH &&
                magazineRingCount != MAGAZINE_RING_COUNT.ZERO) {
            magazineServo.setPower(0.3);
            //TODO : AMJAD : SET TIMER TO EXIT
            while (!magazineLaunchTouchSensor.isPressed() /*&& timer.time() < 2000*/) { }
            magazineServo.setPower(0.05);
            //senseMagazinePosition();
            magazinePosition = MAGAZINE_POSITION.AT_LAUNCH; // Over ride the sense with state set
        }

        if (magazinePosition == MAGAZINE_POSITION.AT_LAUNCH){
            return true;
        } else {
            return false;
        }
    }

    public boolean moveMagazineToCollect() {
        //senseMagazinePosition();
        //senseMagazineRingStatus();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        if (/*magazinePosition != MAGAZINE_POSITION.AT_ERROR &&*/
                magazinePosition != MAGAZINE_POSITION.AT_COLLECT &&
                magazineRingCount != MAGAZINE_RING_COUNT.THREE) {
            magazineServo.setPower(-0.2);
            //TODO : AMJAD : SET TIMER TO EXIT
            while (!magazineCollectTouchSensor.isPressed() /*&& timer.time() < 4000*/)  {
            }
            magazineServo.setPower(0.0);
            //senseMagazinePosition();
            magazinePosition = MAGAZINE_POSITION.AT_COLLECT; // Over ride the sense with state set
        }

        if (magazinePosition == MAGAZINE_POSITION.AT_COLLECT){
            return true;
        } else {
            return false;
        }
    }

    //TODO : AMJAD : Incomplete
    public void senseMagazineRingStatus() {
         magazine_distance = ((DistanceSensor) magazineRingSensor).getDistance(DistanceUnit.CM);

        if((magazine_distance > RING_THREE_DISTANCE) && (magazine_distance < RING_TWO_DISTANCE - 0.2)){
            magazineRingCount = MAGAZINE_RING_COUNT.THREE;
            turnMagazineBeaconWhite();
        } else if((magazine_distance > RING_TWO_DISTANCE) && (magazine_distance < RING_ONE_DISTANCE - 0.2)){
            magazineRingCount = MAGAZINE_RING_COUNT.TWO;
            turnMagazineBeaconTeal();
        } else if((magazine_distance > RING_ONE_DISTANCE) && (magazine_distance < RING_NONE_DISTANCE - 0.05)){
            magazineRingCount = MAGAZINE_RING_COUNT.ONE;
            turnMagazineBeaconPurple();
        } else if((magazine_distance > RING_NONE_DISTANCE)) {
            magazineRingCount = MAGAZINE_RING_COUNT.ZERO;
            turnMagazineBeaconOff();
        }

    }

    //For Telemetry use
    public MAGAZINE_RING_COUNT getMagazineRingCount() {
        //senseMagazineRingStatus();
        return magazineRingCount;
    }

    public MAGAZINE_POSITION getMagazinePosition(){
        senseMagazinePosition();
        return magazinePosition;
    }

    public boolean isMagazineFull(){
        if (getMagazineRingCount() == MAGAZINE_RING_COUNT.THREE){
            return true;
        } else {
            return false;
        }
    }

    public boolean isMagazineEmpty(){
        if (getMagazineRingCount() == MAGAZINE_RING_COUNT.ZERO){
            return true;
        } else {
            return false;
        }
    }

    public void shakeMagazine(int timeInMilliseconds){
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime shaketimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        shaketimer.reset();
        while (shaketimer.time() < timeInMilliseconds) {
            timer.reset();
            while (timer.time() < 50) {
                magazineServo.setPower(0.2);
            }
            timer.reset();
            while (timer.time() < 50) {
                magazineServo.setPower(0.2);
            }
        }
    }

}
