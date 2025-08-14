// needs updating
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

import java.util.concurrent.TimeUnit;

public class GyroOdometry {
//    // Subsystems used for getting encoder and IMU data
//    private OdometrySubsystem odometrySubsystem;
//    private IMUSubsystem imuSubsystem;
//
//    // Total encoder values for front and side (cumulative tracking)
//    public double totalsEncoder = 0;
//    public double totalfEncoder = 0;
//
//    // Temporary integration values (may be used for global velocities)
//    public double tempXIntegrate = 0;
//    public double tempYIntegrate = 0;
//
//    // Robot position in global coordinate frame
//    public double x = 0;
//    public double y = 0;
//
//    // Robot heading in radians
//    public double theta = 0;
//    public double theta2 = 0;
//
//    // Displacement values for current iteration
//    public double dx;
//    public double dy;
//
//    // Velocities in global and local frames
//    public double vxGlobal = 0;
//    public double vyGlobal = 0;
//    public double vxLocal = 0;
//    public double vyLocal = 0;
//
//    // Angular velocity
//    public double vTheta = 0;
//
//    // Timer used for timing each loop cycle
//    public ElapsedTime time;
//
//    // Duration of the last loop
//    public double loopTime;
//
//    // Encoder readings
//    private double sEncoderf; // side (right or left depending on configuration)
//    private double bEncoderf; // back encoder
//    private double lEncoderf; // left encoder
//
//    // Initial encoder readings
//    private double sEncoderi = 0;
//    private double bEncoderi = 0;
//    private double lEncoderi = 0;
//
//    // Constant representing 2π (for angle normalization)
//    private double twoPi = Math.PI * 2;
//
//    // Temporary values for debugging/visualization
//    public double tempX;
//    public double tempY;
//
//    // Change in angle
//    public double dTheta;
//
//    // Conversion factor: centimeters per encoder tick
//    private double dc = odometryCir / odometryTick;
//
//    // Odometry offset correction constants
//    public double dxc = SideOdometryToCentre * Math.cos(sideOdometryAngleFromCentre);
//    public double dyc = lengthFromOdometrySideToFront * Math.cos(frontOdometryAngleFromCentre);
//
//    // Intermediate variable for holding heading during calculations
//    private double thetaTemp;
//
//    // Test/debug values for intermediate math
//    public double testx;
//    public double testx2;
//    public double testy;
//    public double testy2;
//
//    // Constructor to initialize subsystems and timer
//    public GyroOdometry(OdometrySubsystem odometrySubsystem, IMUSubsystem imuSubsystem) {
//        this.odometrySubsystem = odometrySubsystem;
//        this.imuSubsystem = imuSubsystem;
//        time = new ElapsedTime();
//    }
//
//    // Standard odometry processing with gyro correction
//    public void process() {
//        time.reset(); // Reset loop timer
//
//        // Update angle and angular velocity from IMU
//        theta = imuSubsystem.Theta;
//        dTheta = imuSubsystem.dTheta;
//        vTheta = imuSubsystem.vTheta;
//
//        // Update total encoder counts
//        totalfEncoder += bEncoderf;
//        totalsEncoder += sEncoderf;
//
//        // Get new encoder values from odometry subsystem
//        sEncoderf = odometrySubsystem.leftEncoder(); // side encoder
//        bEncoderf = odometrySubsystem.backEncoder(); // back encoder
//
//        // Calculate displacement in robot's local frame
//        dx = (sEncoderf - sEncoderi) * dc - dxc * dTheta;
//        dy = (bEncoderf - bEncoderi) * dc + dyc * dTheta;
//
//        // Transform local displacements to global frame
//        x += dx * Math.cos(theta) - dy * Math.sin(theta);
//        y += dx * Math.sin(theta) + dy * Math.cos(theta);
//
//        // Save intermediate values for telemetry/debug
//        testx = (bEncoderf - bEncoderi) * dc;
//        testx2 = (sEncoderf - sEncoderi) * dc;
//        testy = dxc * dTheta;
//        testy2 = dyc * dTheta;
//
//        // Integrate temporary velocities
//        tempXIntegrate += tempX * time.time();
//        tempYIntegrate += tempY * time.time();
//
//        // Calculate global and local velocities
//        vxGlobal = tempX / time.time();
//        vyGlobal = tempY / time.time();
//        vxLocal = dx / time.time();
//        vyLocal = dy / time.time();
//
//        // Store encoder values for next loop
//        sEncoderi = sEncoderf;
//        bEncoderi = bEncoderf;
//
//        // Record loop duration
//        loopTime = time.time();
//    }
//
//    // Uses only the OdometrySubsystem's values (not IMU)
//    public void odometryProcess() {
//        odometrySubsystem.process();
//        x = odometrySubsystem.x;
//        y = odometrySubsystem.y;
//        theta = odometrySubsystem.theta;
//    }
//
//    // Combines gyro and odometry data for more accurate localization
//    public void combinedProcess() {
//        time.reset(); // Reset timer for velocity calculations
//
//        // Read encoder values
//        lEncoderf = odometrySubsystem.leftEncoder();
//        sEncoderf = odometrySubsystem.rightEncoder();
//        bEncoderf = odometrySubsystem.backEncoder();
//
//        // Calculate local dx using side encoders
//        dx = odometrySubsystem.dxc * ((lEncoderf - lEncoderi) + (sEncoderf - sEncoderi));
//
//        // Estimate change in angle from side encoders
//        dTheta = odometrySubsystem.dThetac * ((sEncoderf - sEncoderi) - (lEncoderf - lEncoderi));
//
//        // Update heading using IMU
//        theta = imuSubsystem.getTheta();
//
//        // Calculate dy with correction for angular motion
//        dy = (odometrySubsystem.dyc * (bEncoderf - bEncoderi)) - (lengthFromOdometrySideToFront * dTheta);
//
//        // Transform to global coordinates
//        x += dx * Math.cos(theta) + dy * Math.sin(theta);
//        y += -dx * Math.sin(theta) + dy * Math.cos(theta);
//
//        // Compute velocities in global and local frames
//        vxGlobal = tempX / time.time(TimeUnit.SECONDS);
//        vyGlobal = tempY / time.time(TimeUnit.SECONDS);
//        vxLocal = dx / time.time(TimeUnit.SECONDS);
//        vyLocal = dy / time.time(TimeUnit.SECONDS);
//        vTheta = dTheta / time.time(TimeUnit.SECONDS);
//
//        // Store current encoder values for next loop
//        sEncoderi = sEncoderf;
//        lEncoderi = lEncoderf;
//        bEncoderi = bEncoderf;
//    }
//
//    // Updates only angle-related variables from IMU
//    public void angleProcess() {
//        theta = imuSubsystem.Theta;
//        dTheta = imuSubsystem.dTheta;
//        vTheta = imuSubsystem.vTheta;
//    }
//
//    // Returns the corrected heading using IMU and zero offset
    public double getAngle() {
        //return imuSubsystem.angleZ() - imuSubsystem.cTheta;
        return 0.0;
    }
//
//    // Returns the absolute angle directly from the IMU
//    public double getAngle2() {
//        return imuSubsystem.getTheta();
//    }
}
