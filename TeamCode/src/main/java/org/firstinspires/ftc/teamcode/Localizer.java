package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class Localizer {

     private GoBildaPinpointDriver gbpd;
     /**
     * Creates a object to track positon and heading. Currently only supports
     * the GoBilda Pidpoint Driver odometry system. Future development may 
     * include other localization methods, including sensor fusion of multiple 
     * localizers using Kalman filters.
     * 
     */
     public Localizer(HardwareMap hm, String name){
          if (name.contains("gbpoc")){
               final double X_OFFSET = -84.0;  // how sideways from the center of the robot is the X (forward) pod? Left increases
               final double Y_OFFSET = -168.0; // how far forward from the center of the robot is the Y (strafe) pod? forward increases
               gbpd = hm.get(GoBildaPinpointDriver.class, "gbpoc");
               gbpd.setOffsets(X_OFFSET, Y_OFFSET, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
               gbpd.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
               gbpd.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
               gbpd.resetPosAndIMU();
          }
          // TODO add other localization sensors
     }
     
     public void update(){
          if (gbpd != null) gbpd.update();
     }
     
     public Pose2D getPosition(){
          Pose2D gbpdPose = null;
          if (gbpd != null){
            gbpdPose = gbpd.getPosition();
          }
          // TODO fuse multiple sensor readings
          Pose2D fused  = gbpdPose;
          return fused;
     }
}