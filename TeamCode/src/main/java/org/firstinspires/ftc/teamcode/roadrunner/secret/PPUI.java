package org.firstinspires.ftc.teamcode.roadrunner.secret;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.Vector;
@Config
public class PPUI {
  ArrayList<Waypoint> path;

  ArrayList<Pose2d> ogPoints;
  SampleMecanumDrive roadrun;
  double MAX_VEL = 80;
  double MAX_ACCEL = 50;
  double populateThresh = 6;
  int rev = 1;

  double output=0;

  int closestPoint = 0;

  Vector2d lookAheadPoint;

  double curvature = 0;

  double lastTime=0;

  double lastL = 0;
  double lastR = 0;

  int side = 1;
public static   double TRACK_WIDTH = 15;
  public static double kV = 0.0054, kA = 0.001, kP = 0.0003;


  public PPUI(SampleMecanumDrive road) {
    roadrun = road;
    ogPoints = new ArrayList<>();
  }

  public void populatePath() {
      for(int i=0;i<path.size();i++){
          ogPoints.add(path.get(i).pose);
      }
    for (int i = 1; i < path.size(); i++) {
      Pose2d pose1 = path.get(i).pose, pose2 = path.get(i - 1).pose;
      if (pose1.vec().distTo(pose2.vec()) > populateThresh) {
        path.add(
            i,
            new Waypoint(
                new Pose2d(pose1.vec().plus(pose2.vec()).times(0.5), pose1.getHeading()),
                path.get(i).lookAhead));
        i--;
      }
    }
  }

  public void calcDistances() {
    for (int i = 1; i < path.size(); i++) {
      Pose2d pose1 = path.get(i).getPose(), pose2 = path.get(i - 1).getPose();
      path.get(i).setDistance(path.get(i - 1).getDistance() + pose1.vec().distTo(pose2.vec()));
    }
  }

  public double curvatureAtPoint(int i){
      Pose2d pose1 = path.get(i).getPose(), pose2 = path.get(i - 1).getPose(), pose3 = path.get(i+1).getPose();
      double x1 = pose1.getX(), x2 = pose2.getX(), y1 = pose1.getY(), y2 = pose2.getY(),x3=pose3.getX(), y3 = pose3.getY();
      if(x1==x2){
          x2+=0.0001;
      }
      double k1 = 0.5*(x1*x1+y1*y1-x2*x2-y2*y2)/(x1-x2);
      double k2 = (y1-y2)/(x1-x2);
      double b = 0.5*(x2*x2-2*x2*k1+y2*y2-x3*x3+2*x3*k1-y3*y3)/(x3*k2-y3+y2-x2*k2);
      if(Double.isNaN(b)){
          b=10000;
      }
      double a =k1-k2*b;
      double r = sqrt((x1-a)*(x1-a)+(y1-b)*(y1-b));
      if(Double.isNaN(r)){
          return 0;
      }
      if(r==0){
          return 10000;
      }
      return 1/r;
  }
  public void calcVelos(){
      for(int i=1;i<path.size()-1;i++){
          Pose2d pose1 = path.get(i).getPose(), pose2 = path.get(i - 1).getPose();
          double vi = path.get(i-1).getVelocity();
          double a = max(MAX_ACCEL-vi*vi*curvatureAtPoint(i),0);
          double d = pose1.vec().distTo(pose2.vec());
          path.get(i).setVelocity(sqrt(vi*vi+2*a*d));
          packet.put("pose"+i, path.get(i).pose);
          packet.put("1vel"+i, path.get(i).getVelocity());

      }
      for(int i=path.size()-2; i>0;i--){
          Pose2d pose1 = path.get(i).getPose(), pose2 = path.get(i - 1).getPose();
          double vi = path.get(i+1).getVelocity();
          double a = max(MAX_ACCEL-vi*vi*curvatureAtPoint(i),0);
          double d = pose1.vec().distTo(pose2.vec());
          path.get(i).setVelocity(min(path.get(i).getVelocity(),sqrt(vi*vi+2*a*d)));
          packet.put("vel"+i, path.get(i).getVelocity());
      }
  }

  public void limitRate(double input){
      output=output+Math.signum(input-output)*min(abs(input-output), 1);
//      if(abs(input)<abs(output)){
//          output = input;
//      }
  }

  public void calcClosestPoint(){
      double minDist = currentPose.vec().distTo(path.get(closestPoint).pose.vec());
      for(int i=closestPoint+1; i<path.size();i++){
          double newDist = currentPose.vec().distTo(path.get(i).pose.vec());
          if(newDist<minDist){
              minDist=newDist;
              closestPoint=i;
          }
          else{
              break;
          }
      }
  }
  public void calcLookaheadPoint(){
    for (int i = max(closestPoint - 1,0); i < path.size() - 1; i++) {
      Vector2d E = path.get(i).pose.vec(), L = path.get(i+1).pose.vec(), C = currentPose.vec(), d = L.minus(E), f = E.minus(C);
      double r = path.get(closestPoint).getLookAhead();
        double a = d.dot(d);
        double b = 2*f.dot(d);
        double c = f.dot(f) - r*r;
        double discriminant = b*b - 4*a*c;
        if (discriminant < 0) {
//            lookAheadPoint=path.get(closestPoint).pose.vec();
        }else{
            discriminant = sqrt(discriminant);
            double t1 = (-b - discriminant)/(2*a);
            double t2 = (-b + discriminant)/(2*a);
            if (t1 >= 0 && t1 <=1){
                lookAheadPoint=E.plus(d.times(t1));
                double aa = -tan(currentPose.getHeading()*rev);
                double bb = 1;
                double cc = -aa*currentPose.getX()-currentPose.getY();
                double x = abs(aa*lookAheadPoint.getX()+bb*lookAheadPoint.getY()+cc)/(aa*aa+bb*bb);
                double l =r;
                curvature= (2*x)/l*l;
                packet.put("curvature", curvature);
                packet.put("lookAheadPoint", lookAheadPoint);
                side = (int) signum(sin(currentPose.getHeading()*rev)*(lookAheadPoint.getX()-currentPose.getX())-cos(currentPose.getHeading()*rev)*(lookAheadPoint.getY()-currentPose.getY()));

            }
            else if (t2 >= 0 && t2 <=1){
                lookAheadPoint=E.plus(d.times(t2));
                double aa = -tan(currentPose.getHeading()*rev);
                double bb = 1;
                double cc = -aa*currentPose.getX()-currentPose.getY();
                double x = abs(aa*lookAheadPoint.getX()+bb*lookAheadPoint.getY()+cc)/(aa*aa+bb*bb);
                double l =r;
                curvature= (2*x)/l*l;
                packet.put("curvature", curvature);
                packet.put("lookAheadPoint", lookAheadPoint);
                side = (int) signum(sin(currentPose.getHeading()*rev)*(lookAheadPoint.getX()-currentPose.getX())-cos(currentPose.getHeading()*rev)*(lookAheadPoint.getY()-currentPose.getY()));
            }
            else{
//                lookAheadPoint=path.get(closestPoint).pose.vec();
            }
        }


    }

  }

  public double[][] waypointsToArray(){
      double[][] copy = new double[path.size()][2];
      for(int i=0;i<path.size();i++){
          copy[i][0] = path.get(i).pose.getX();
          copy[i][1] = path.get(i).pose.getY();
      }
      return copy;
  }

  public ArrayList<Waypoint> arrayToWaypoint(double [][] p_path){
      ArrayList<Waypoint> waypoints = new ArrayList<>();
      for(int i=0; i<p_path.length;i++){
          waypoints.add(new Waypoint(new Pose2d(p_path[i][0], p_path[i][1],0), path.get(i).getLookAhead()));
      }
      return waypoints;
  }
    public static double[][] doubleArrayCopy(double[][] arr)
    {

        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];

        for(int i=0; i<arr.length; i++)
        {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];

            //Copy Contents
            for(int j=0; j<arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }

        return temp;

    }


    public double[][] smoother(double[][] path, double a, double b, double tolerance){
//copy array
        double[][] newPath = doubleArrayCopy(path);
        double change = tolerance;
        while(change >= tolerance)
        {
            change = 0.0;
            for(int i=1; i<path.length-1; i++)
                for(int j=0; j<path[i].length; j++)
                {
                    double aux = newPath[i][j];
                    newPath[i][j] += a * (path[i][j] - newPath[i][j]) + b *
                            (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
        }
        return newPath;
    }
  public void followPath(ArrayList<Waypoint> points, int reverse) {
      rev = reverse;
      packet.put("among0", true);
      if (ogPoints.isEmpty()||points.size()!=ogPoints.size()||!points.get(points.size()-1).pose.equals(ogPoints.get(ogPoints.size()-1))) {
        packet.put("among", true);
        output=0;
      path = points;
      closestPoint=1;
      lookAheadPoint = path.get(0).pose.vec();
      ogPoints=new ArrayList<>();
      lastTime = time-0.01;
      populatePath();
      double b = 0.75, a = 1-b;
      path = arrayToWaypoint(smoother(waypointsToArray(),a,b,0.001));
      packet.put("among2",true);
      calcDistances();
        packet.put("among3",true);

        calcVelos();
        lastL = 0;
      lastR = 0;
    }
      packet.put("among0", true);
      calcClosestPoint();
    calcLookaheadPoint();
    double turner=0;
//    if(lookAheadPoint.equals(path.get(path.size()-1).pose.vec())){
//        turner = kV * (ogPoints.get(ogPoints.size()-1).getHeading()*rev-currentPose.getHeading()*reverse)*TRACK_WIDTH;
//
//    }
    limitRate(path.get(closestPoint).getVelocity());
      packet.put("input", path.get(closestPoint).getVelocity());
      packet.put("closest", closestPoint);

      double targetVel = output*reverse;
      packet.put("output", targetVel);
    double C = curvature*side*rev;
      packet.put("C", C);

      double T = TRACK_WIDTH;
    double L = targetVel*(2+C*T)/2;
    double R = targetVel*(2-C*T)/2;
    double FFl = kV * L /*+ kA*(L-lastL)/(time-lastTime)*/;
    double FFr = kV*R /*+ kA*(R-lastR)/(time-lastTime)*/;
    double FBl = kP*(L-roadrun.getTracker().getWheelVelocities().get(0));
    double FBr = kP*(R-roadrun.getTracker().getWheelVelocities().get(1));
    lastL = L;
    lastR = R;
    FBl = 0;
    FBr = 0;
    packet.put("FFl", FFl);
      packet.put("FBl", FBl);
      packet.put("FFr", FFr);
      packet.put("FBr", FBl);

      roadrun.setMotorPowers(FFl+FBl-turner,FFl+FBl-turner, FFr+FBr+turner,FFr+FBr+turner);
    lastTime = time;


  }
}
