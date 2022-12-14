package org.firstinspires.ftc.teamcode.Functions;


import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.teamcode.Functions.AccelerationDetector;
import java.io.IOException;


public class AccelerationReader {

    // Aceasta functie se initializeaza astfel:
    // accelerationReader = new AccelerationReader(hardwareMap.get(BNO055IMUImpl.class, "imu"));

    /*
    -------------------------------------
    TEMPLATE CUM SA FOLOSESTI CLASA ASTA:
    -------------------------------------
    int Distance = <--- aici pui distanta
    while(accelerationReader.WaitForDestination(Distanta, getRunTime())
    {
        move.MoveFull(1);
    }
    move.MoveStop(); // aceasta linie e doar de siguranta.

    */

    AccelerationDetector accelerationDetector;

    public AccelerationReader(BNO055IMU gyro){

        try {
            accelerationDetector = new AccelerationDetector(gyro);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
    double lastRunTime = 0;
    double currentDistance=0;
    double currentVelocity=0;
    /// primeste distanta
    public boolean WaitForDestination(int targetDistance, double currentTime) {
        if (targetDistance <= currentDistance) {
            /// resetam variabilele
            currentDistance = 0;
            lastRunTime = 0;
            currentVelocity = 0;
            return false;
        }
        /// calculam distanta parcursa de robot


        if (lastRunTime == 0) {
            lastRunTime = currentTime;
        } else {
            ///  (a*t^2)/2
            /// a = currentAcceleration
            double currentAcceleration = accelerationDetector.ReturnX() + accelerationDetector.ReturnY();
            double deltaTime = currentTime - lastRunTime;
            lastRunTime = currentTime;
            currentDistance += (currentAcceleration * deltaTime * deltaTime) / 2;/// (a*t^2)/2

        }
        return true;

    }

        /// Această funcție e folosită pentru returnarea variabilelor folosite în AccelerationReader sub formă de Strings

        public String DataDebug(double currentTime)
        {
            return "lastRunTime: " + lastRunTime + "\ncurrentTime: " + currentTime + "\ncurrentVelocity: " + currentVelocity + "\ncurrentDistance: " + currentDistance;
        }


    }


    /*else if(currentVelocity==0){
        currentVelocity = (currentTime - lastRunTime)*;
    }*/
