package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;

public class RobotTracker implements Runnable{
    
    public static double[] output = {0, 0};
    
    final int ULTRASONIC_ANGLE_TOLERANCE = 30; // furthest angle from perpendicular that ultra sonic readings will be made
    final int ULTRASONIC_DISTANCE_TOLERANCE = 3; // in
    final int MIN_DISTANCE = 7; // in
    final int MAX_DISTANCE = 175; // in
    final int FIELD_X_SIZE = 144; // 12 ft
    final int FIELD_Y_SIZE = 96; // 8 ft
    
    double[] ultrasonicPosition = {0, 0, 0, 0};  // index starts with right sensor and moves counter-clockwise
    int forwardsAxis;
    
    double oldXEncoder = 0;
    double oldYEncoder = 0;
    double newXEncoder;
    double newYEncoder;
    
    double movementAngle;
    double movementDistance;
    double totalDistance = 0;
    RobotHardware H;
    LinearOpMode opMode;
    
    RobotTracker(RobotHardware H/*, LinearOpMode opMode*/) {
        
        this.H = H;
        //this.opMode = opMode;
    }
    
    void setInitialPosition(int x, int y) {
        output[0] = x;
        output[1] = y;
    }
    
    void setInitialPosition() {
        output[1] = 6;
        
    }
    
    public void run() {
    
        while (!H.isStopRequested()) {
    
            Iterate();
    
        }
        
    }
    
    double[] Iterate() {
        
        newXEncoder = ((double)H.xEncoder.getCurrentPosition() * 3.14159 * 1.5205) / (900 * 4);
        newYEncoder = ((double)H.yEncoder.getCurrentPosition() * 3.14159 * 1.5205) / (900 * 4);
    
        double dx = newXEncoder - oldXEncoder;
        double dy = newYEncoder - oldYEncoder;
        
        movementAngle = Math.atan2(dy,dx);
        movementDistance = Math.hypot(dx,dy);
        totalDistance += movementDistance;
        
        output[0] += movementDistance*Math.cos(movementAngle + Math.toRadians(H.heading));
        output[1] += movementDistance*Math.sin(movementAngle + Math.toRadians(H.heading));
    
        oldXEncoder = newXEncoder;
        oldYEncoder = newYEncoder;
    
        // find angle away from any axis. if under tolerance measure using ultrasonic sensors
        if (H.newRangeDataFlag) {
            if (Math.abs(((H.heading + 180 + 45) % 90) - 45) < ULTRASONIC_ANGLE_TOLERANCE && movementDistance < 0.2) {
                forwardsAxis = (int)((H.heading + 180 + 45) / 90);
                if (forwardsAxis == 4) forwardsAxis = 0;
                for (int i = 0; i < 4; i++) {
                    int adjustedIndex = i + forwardsAxis;
                    if (adjustedIndex > 3) adjustedIndex -= 4;
                    ultrasonicPosition[adjustedIndex] = measureUltrasonic(i);
                }
                //opMode.telemetry.addData("MB","0: (%2f), 1: (%2f), 2: (%2f), 3: (%2f)", H.range[0], H.range[1], H.range[2], H.range[3]);
                //opMode.telemetry.addData("UP","right: (%2f), forward: (%2f), left: (%2f), back: (%2f)", ultrasonicPosition[0], ultrasonicPosition[1], ultrasonicPosition[2], ultrasonicPosition[3]);
                ultrasonicPosition[0] = FIELD_X_SIZE - ultrasonicPosition[0];
                ultrasonicPosition[1] = FIELD_Y_SIZE - ultrasonicPosition[1];


                // if the two opposing sensors read within a value of each other average to find distance otherwise use the lower value
                if (Math.abs(ultrasonicPosition[0] - ultrasonicPosition[2]) < ULTRASONIC_DISTANCE_TOLERANCE) {
                    ultrasonicPosition[0] = 0.5 * (ultrasonicPosition[0] + ultrasonicPosition[2]);
                } else {
                    ultrasonicPosition[0] = Math.min(ultrasonicPosition[0], ultrasonicPosition[2]);
                }
                if (Math.abs(ultrasonicPosition[1] - ultrasonicPosition[3]) < ULTRASONIC_DISTANCE_TOLERANCE) {
                    ultrasonicPosition[1] = 0.5 * (ultrasonicPosition[1] + ultrasonicPosition[3]);
                } else {
                    ultrasonicPosition[1] = Math.min(ultrasonicPosition[1], ultrasonicPosition[3]);
                }

                if (Math.abs(output[0] - ultrasonicPosition[0]) < ULTRASONIC_DISTANCE_TOLERANCE) {
                    output[0] = 0.5 * (output[0] + ultrasonicPosition[0]);
                }
                if (Math.abs(output[1] - ultrasonicPosition[1]) < ULTRASONIC_DISTANCE_TOLERANCE) {
                    output[1] = 0.5 * (output[1] + ultrasonicPosition[1]);
                }

            }
            H.newRangeDataFlag = false;

//            opMode.telemetry.addData("distance","x: (%2f), y: (%2f)", ultrasonicPosition[0], ultrasonicPosition[1]);
//            opMode.telemetry.addData("pos","x: (%2f), y: (%2f)", output[0], output[1]);
//            opMode.telemetry.addData("heading", H.heading);
//            opMode.telemetry.update();
        }
    
        Log.d("tracker", "Iterate: " + output[0] + ", " + output[1]);
        
        return output;
    }
    
    double measureUltrasonic (int sensorIndex) {
        if (H.range[sensorIndex] > MIN_DISTANCE && H.range[sensorIndex] < MAX_DISTANCE) {
            return H.range[sensorIndex];
        }
        return 99999;
    }
    
    void reset() {
        output[0] = 0;
        output[1] = 0;
    }
    
}
