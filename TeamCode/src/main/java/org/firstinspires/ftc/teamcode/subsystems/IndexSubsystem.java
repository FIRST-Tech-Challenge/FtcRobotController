package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.Sensor;
import com.technototes.library.subsystem.motor.MotorSubsystem;

import java.util.function.IntSupplier;

/** Index Subsystem
 *
 */
public class IndexSubsystem extends MotorSubsystem<Motor<?>> {

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

    public IndexState state;

    public IndexSubsystem(Motor motor){
        super(motor);
//        state = IndexState.calculateFromDigitalSensor();
    }

    public void sendToShooter(){

    }

}
