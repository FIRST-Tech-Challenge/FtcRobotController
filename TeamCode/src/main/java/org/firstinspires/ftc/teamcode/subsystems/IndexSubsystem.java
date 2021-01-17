package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.DigitalSensor;
import com.technototes.library.hardware.sensor.RangeSensor;
import com.technototes.library.hardware.sensor.Sensor;
import com.technototes.library.subsystem.motor.MotorSubsystem;
import com.technototes.logger.Stated;

import org.jetbrains.annotations.Nullable;

import java.util.function.IntSupplier;

/** Index Subsystem
 *
 */
public class IndexSubsystem extends MotorSubsystem<Motor<?>> implements Stated<IndexSubsystem.IndexState> {

    public enum IndexState{
        EMPTY(0), ONE_RING(1), TWO_RINGS(2), FULL(3);
        public int numRings;
        IndexState(int rings){
            numRings = rings;
        }
        public int getNumRings(){
            return numRings;
        }

        public static IndexState calculateFromRangeSensor(double distance){
            if (distance > 40) return EMPTY;
            if (distance > 30) return ONE_RING;
            if (distance > 20) return TWO_RINGS;
            return FULL;
        }
        public static IndexState calculateFromDigitalSensor(boolean sens1, boolean sens2, boolean sens3){
            if (sens1) return FULL;
            if (sens2) return TWO_RINGS;
            if (sens3) return ONE_RING;
            return EMPTY;
        }
    }

    @Nullable
    public RangeSensor rangeSensor;
    @Nullable
    public DigitalSensor digitalSensor1, digitalSensor2, digitalSensor3;

    private boolean sensorConfiguration; //true is 1 range, false is 3 digital

    public IndexSubsystem(Motor motor, RangeSensor sensor){
        super(motor);
        sensorConfiguration = true;
        rangeSensor = sensor;
    }

    public IndexSubsystem(Motor motor, DigitalSensor sensor1, DigitalSensor sensor2, DigitalSensor sensor3){
        super(motor);
        sensorConfiguration = false;
        digitalSensor1 = sensor1;
        digitalSensor2 = sensor2;
        digitalSensor3 = sensor3;
    }

    public void sendToShooter(){
        //TODO
    }



    @Override
    public IndexState getState(){
        return getSensorConfiguration() ? IndexState.calculateFromRangeSensor(rangeSensor.getSensorValue()) : IndexState.calculateFromDigitalSensor(
                digitalSensor1.getSensorValueAsBoolean(), digitalSensor2.getSensorValueAsBoolean(), digitalSensor3.getSensorValueAsBoolean());
    }

    public boolean getSensorConfiguration() {
        return sensorConfiguration;
    }
    public boolean isFull(){
        return getState() != IndexState.FULL;
    }

}
