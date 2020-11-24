package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Color beacon code from https://modernroboticsinc.com/product/color-beacon/

public class Magazine {
    //public ElapsedTime magazineTimer;
    public CRServo magazineServo;
    public NormalizedColorSensor magazineRingSensor;


    public I2cDevice magazineColorBeacon;
    public I2cDeviceSynch magazineColorBreader;

    public TouchSensor magazineLaunchTouchSensor;
    public TouchSensor magazineCollectTouchSensor;

    LinearOpMode opModepassed;

    //TODO : AMJAD : Magazine ring distance values to be set correctly
    //Distance measure is float. Need to convert to float than int.

    public static final double RING_NONE_DISTANCE = 3.4;
    public static final double RING_ONE_DISTANCE = 3.0;
    public static final double RING_TWO_DISTANCE = 2.0;
    public static final double RING_THREE_DISTANCE = 0.5;

    public double magazine_distance;


    public enum MAGAZINE_POSITION {
        MAGAZINE_AT_COLLECT,
        MAGAZINE_AT_LAUNCH,
        MAGAZINE_AT_ERROR
    }

    public enum MAGAZINE_RING_COUNT {
        MAGAZINE_RINGS_0,
        MAGAZINE_RINGS_1,
        MAGAZINE_RINGS_2,
        MAGAZINE_RINGS_3
    }

    public enum MAGAZINE_TOUCH_SENSORS_STATE {
        LAUNCH_TS_PRESSED,
        LAUNCH_TS_NOT_PRESSED,
        COLLECT_TS_PRESSED,
        COLLECT_TS_NOT_PRESSED,
        TS_ERROR
    }

    public MAGAZINE_RING_COUNT magazineRingCount = MAGAZINE_RING_COUNT.MAGAZINE_RINGS_0;
    public MAGAZINE_POSITION magazinePosition = MAGAZINE_POSITION.MAGAZINE_AT_COLLECT;

    public Magazine(HardwareMap hardwareMap) {
        magazineServo = hardwareMap.crservo.get("mgz_servo");
        magazineRingSensor = hardwareMap.get(NormalizedColorSensor.class, "mgz_ring_sensor");

        //magazineColorBeacon = hardwareMap.i2cDevice.get("mgz_beacon");
       // magazineColorBreader = new I2cDeviceSynchImpl(magazineColorBeacon, I2cAddr.create8bit(0x4c), false);
        //magazineColorBreader.engage();

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
            magazinePosition = MAGAZINE_POSITION.MAGAZINE_AT_LAUNCH;
        } else if (magazine_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_PRESSED) {
            magazinePosition = MAGAZINE_POSITION.MAGAZINE_AT_COLLECT;
        } else if (magazine_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.TS_ERROR){
            magazinePosition = MAGAZINE_POSITION.MAGAZINE_AT_ERROR;
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

        //magazineColorBreader.write8(4, 0);
    }

    public void turnMagazineBeaconPurple() {

        //magazineColorBreader.write8(4, 5);
    }

    public void turnMagazineBeaconTeal() {

        //magazineColorBreader.write8(4, 6);
    }

    public void turnMagazineBeaconWhite() {

        //magazineColorBreader.write8(4, 7);
    }

    public boolean moveMagazineToLaunch() {
        senseMagazinePosition();
        senseMagazineRingStatus();
        if(magazinePosition != MAGAZINE_POSITION.MAGAZINE_AT_ERROR &&
                magazinePosition != MAGAZINE_POSITION.MAGAZINE_AT_LAUNCH &&
                magazineRingCount != MAGAZINE_RING_COUNT.MAGAZINE_RINGS_0) {

        while (!magazineLaunchTouchSensor.isPressed()){
                magazineServo.setPower(0.1);
            }
            magazineServo.setPower(0);

            senseMagazinePosition();
        }

        if (magazinePosition == MAGAZINE_POSITION.MAGAZINE_AT_LAUNCH){
            return true;
        } else {
            return false;
        }
    }

    public boolean moveMagazineToCollect() {
        senseMagazinePosition();
        senseMagazineRingStatus();
        if (magazinePosition != MAGAZINE_POSITION.MAGAZINE_AT_ERROR &&
                magazinePosition != MAGAZINE_POSITION.MAGAZINE_AT_COLLECT &&
                magazineRingCount != MAGAZINE_RING_COUNT.MAGAZINE_RINGS_3) {

            while (!magazineCollectTouchSensor.isPressed()) {
                magazineServo.setPower(-0.1);
            }
            magazineServo.setPower(0);

            senseMagazinePosition();
        }

        if (magazinePosition == MAGAZINE_POSITION.MAGAZINE_AT_COLLECT){
            return true;
        } else {
            return false;
        }
    }

    //TODO : AMJAD : Incomplete
    public void senseMagazineRingStatus() {
         magazine_distance = ((DistanceSensor) magazineRingSensor).getDistance(DistanceUnit.CM);

        if((magazine_distance > RING_THREE_DISTANCE) && (magazine_distance < RING_TWO_DISTANCE - 0.2)){
            magazineRingCount = MAGAZINE_RING_COUNT.MAGAZINE_RINGS_3;
            //turnMagazineBeaconWhite();
        } else if((magazine_distance > RING_TWO_DISTANCE) && (magazine_distance < RING_ONE_DISTANCE - 0.2)){
            magazineRingCount = MAGAZINE_RING_COUNT.MAGAZINE_RINGS_2;
            //turnMagazineBeaconTeal();
        } else if((magazine_distance > RING_ONE_DISTANCE) && (magazine_distance < RING_NONE_DISTANCE - 0.05)){
            magazineRingCount = MAGAZINE_RING_COUNT.MAGAZINE_RINGS_1;
            //turnMagazineBeaconPurple();
        } else if((magazine_distance > RING_NONE_DISTANCE)) {
            magazineRingCount = MAGAZINE_RING_COUNT.MAGAZINE_RINGS_0;
            //turnMagazineBeaconOff();
        }

    }

    //For Telemetry use
    public MAGAZINE_RING_COUNT getMagazineRingCount() {
        senseMagazineRingStatus();
        return magazineRingCount;
    }

    public MAGAZINE_POSITION getMagazinePosition(){
        senseMagazinePosition();
        return magazinePosition;
    }

    public boolean isMagazineFull(){
        if (getMagazineRingCount() == MAGAZINE_RING_COUNT.MAGAZINE_RINGS_3){
            return true;
        } else {
            return false;
        }
    }

    public boolean isMagazineEmpty(){
        if (getMagazineRingCount() == MAGAZINE_RING_COUNT.MAGAZINE_RINGS_0){
            return true;
        } else {
            return false;
        }
    }

}
