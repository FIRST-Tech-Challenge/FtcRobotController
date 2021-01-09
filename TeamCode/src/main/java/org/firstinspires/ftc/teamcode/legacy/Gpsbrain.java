// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import java.lang.reflect.Array;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import org.firstinspires.ftc.teamcode.Collect;
// import org.firstinspires.ftc.teamcode.Drive;
// //import org.firstinspires.ftc.teamcode.Find;
// import org.firstinspires.ftc.teamcode.SciLift;

// //The meat. Our magnum opus. The incredible autonomous program.
// //We control the robot using special commands stored in a list.
// //More information on how we control the bot here:
// //https://github.com/HHS-FTC-Robotics-Team/Team-Resources/wiki/States

// public class Gpsbrain extends LinearOpMode {

//   public String state = "rest";
//   public Boolean turning = false;
//   boolean angleIsSeeked = false;
//   double seekAngle = 0;
//   double seekDist = 0;
//   Drive d = null;
//   double globalx = 0;
//   double globaly = 0;
//   double startx = 0;
//   double starty = 0;
//   double globala = 0;
//   double dx = 0;
//   double dy = 0;
//   double theta = 0;
//   double dtheta = 0;
//   double travelled = 0;
//   public double goalclicks = 0;
//   public double relativex = 0;
//   public double relativey = 0;
//   double startclicks = 0;
//   double liftgoalclicks = 0;
//   double liftstartclicks = 0;
//   public int count = 0;
  
  

//   //List of different command sequences===================================================================================

//   // without seeking
//     //blue
//     // public String[] states = new String[]   {"init","forwardTo","lift","turn","collect","forwardTo","strafeTo", "out", "strafeTo","rest"};
//     // private double[] args = new double[]    {0, 1200, 0, 180, 0, 1000, -4000, 0, -2000, 0};
//     // private boolean[] isArgs = new boolean[]{false, true, true, true, false, true, true,false, true, false};
//     //red
//     // public String[] states = new String[]   {"init","forwardTo","lift","turn","collect","forwardTo","strafeTo", "out", "strafeTo","rest"};
//     // private double[] args = new double[]    {0, 1200, 0, 180, 0, 1000, 4000, 0, 2000, 0};
//     // private boolean[] isArgs = new boolean[]{false, true, true, true, false, true, true,false, true, false};


//   // with seeking
//     //blue
//     // public String[] states = new String[]   {"init","forwardTo","oldseek","lift","turn","collect","forwardTo","strafeTo", "out", "strafeTo","rest"};
//     // private double[] args = new double[]    {0, 1200, 0, 0, 180, 0, 1000, -4000, 0, -2000, 0};
//     // private boolean[] isArgs = new boolean[]{false, true, false, true, true, false, true, true,false, true, false};
//     //red
//     // public String[] states = new String[]   {"init","forwardTo","oldseek","lift","turn","collect","forwardTo","strafeTo", "out", "strafeTo","rest"};
//     // private double[] args = new double[]    {0, 1200, 0, 0, 180, 0, 1000, 4000, 0, 2000, 0};
//     // private boolean[] isArgs = new boolean[]{false, true, false, true, true, false, true, true,false, true, false};


//   // Wait and Park
//     //blue
//     // public String[] states = new String[]{"sleep", "forward", "strafeTo", "rest"};
//     // private double[] args = new double[]{24000, 500, -1200, 0};
//     // private boolean[] isArgs = new boolean[]{true, true, true, false};
//     //red
//     // public String[] states = new String[]{"sleep", "forward", "strafeTo", "rest"};
//     // private double[] args = new double[]{2400, 500, 1200, 0};
//     // private boolean[] isArgs = new boolean[]{true, true, true, false};

//   //Don't wait, just park
//     //blue
//     public String[] states = new String[]{"forward", "strafeTo", "rest"};
//     private double[] args = new double[]{50, 2400, 0}; //changed the 800 to 50

//     private boolean[] isArgs = new boolean[]{ true, true, false};
//     //red
//     // public String[] states = new String[]{"forward", "strafeTo", "rest"};
//     // private double[] args = new double[]{500, -1200, 0};
//     // private boolean[] isArgs = new boolean[]{ true, true, false};


//   //======================================================================================================================

//   private BNO055IMU imu = null;
//   private Orientation lastAngles = new Orientation();
//   private double globalAngle, power = 0.30, correction;
//   public SciLift lift = null;
//   Collect collect = null;
//   Find f = null;

//   public Gpsbrain(Drive drive, BNO055IMU acc, Collect c, Find find, SciLift scl) {
//     d = drive;
//     BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//       parameters.mode                = BNO055IMU.SensorMode.IMU;
//       parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//       parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//       parameters.loggingEnabled      = false;
//     imu = acc;
//     imu.initialize(parameters);
//     collect = c;
//     f = find;
//     lift = scl;
//   }

//   public void pop() {
//     count = count + 1;
//   }

//   public void update() {
//     if(states[count] == "init") {
//       globalx = 0;
//       globaly = 0;
//       globala = getAngle(); //we will always correct to this angle
//       d.resetEncoderlf();
//       pop();
//     }
//     if(states[count] == "rest") {
//       d.setPower(0, 0, 0, 0);
//       lift.motor.setPower(0);
//     }
//     if(states[count] == "turn") {
//       if (isArgs[count]) {
//         this.turn(args[count]);
//         isArgs[count] = false;
//       }
//       this.turn();
//     }
//     if(states[count] == "forward"){
//       if (isArgs[count]) {
//         this.forward(args[count]);
//         isArgs[count] = false;
//       }
//       this.forward();
//     }
//     if(states[count] == "forwardTo"){
//       if (isArgs[count]) {
//         this.forwardTo(args[count]);
//         isArgs[count] = false;
//       }
//       this.forward();
//     }
//     if(states[count] == "strafeTo"){
//       if (isArgs[count]) {
//         this.strafeTo(args[count]);
//         isArgs[count] = false;
//       }
//       this.strafe();
//     }
//     if(states[count] == "seek") {
//       //angleIsSeeked = false;
//       if(!angleIsSeeked) {
//         double[] result = f.findSkystoneAngle();
//         seekAngle = result[0];
//         if(result[1] > 0) {
//           seekDist = -600*Math.tan(seekAngle);
//           //strafe(seekDist);
//           angleIsSeeked = true;
//           // if(seekAngle < 0) {
//           //   seekDist = seekDist * -1;
//           // }
//           strafe(seekDist);
//         }
//       } else {
//         //seekAngle =-500;
//         strafe();
//       }
//       // double angle = seekAngle;
//       // double dist = seekDist;
//       // if(angle < 10 && angle > -10) {
//       //   globalx += d.getClickslf();
//       //   d.resetEncoderlf();
//       //   angleIsSeeked = false;
//       //   pop();
//       // } else {
//       //   d.setPower(0, dist/1000, 0, 0);
//       //   globalx += d.getClickslf();
//       //   d.resetEncoderlf();
//       // }
//     }
//     if(states[count] == "oldseek") {
//       if (isArgs[count]) { //runs the first time, the initial check
//         double[] result = f.findSkystoneAngle();
//         double angle = result[0];
//         if (result[1] == 0) {
//           pop(); //pops if theres no block, that way we can at least try for any block.
//         }
//         isArgs[count] = false;
//       } else {
//         double[] result = f.findSkystoneAngle();
//         double angle = result[0];
//         if(angle < 10 && angle > -10) {
//           globalx += d.getClickslf();
//           d.resetEncoderlf();
//           pop();
//         } else {
//           d.setPower(0, 1*angle/20, 0, 0.6);
//           globalx += d.getClickslf();
//           d.resetEncoderlf();
//         }
//       }
//     }
//     if(states[count] == "collect") {
//       if(collect.getDistance() > 10) {
//         d.setPower(1, 0, 0, 0.6);
//         collect.in();
//         globaly += d.getClickslf();
//         d.resetEncoderlf();
//       } else if (collect.getDistance() <= 10) {
//         d.setPower(0, 0, 0, 0);
//         collect.rest();
//         globaly += d.getClickslf();
//         d.resetEncoderlf();
//         pop();
//       }
//     }
//     if(states[count] == "out") {
//       if(collect.getDistance() < 20) {
//         d.setPower(0, 0, 0, 0);
//         collect.out();
//       } else if (collect.getDistance() > 20) {
//         collect.rest();
//         pop();
//       }
//     }
//     if(states[count] == "lift"){
//       if (isArgs[count]) {
//         d.setPower(0, 0, 0, 0);
//         lift.motor.setPower(-0.8);
//         sleep(800);
//         lift.motor.setPower(0);
//         sleep(800);
//         lift.motor.setPower(0.7);
//         sleep(800);
//         lift.motor.setPower(0);
//         pop();
//         isArgs[count] = false;
//       } else {
//         pop();
//       }
//     }
//     if(states[count] == "sleep") {
//       if(isArgs[count]) {
//         double slep = args[count];
//         long bruh = (long)slep * 10;
//         sleep(bruh);
//         isArgs[count] = false;
//         pop();
//       } else {
//         pop();
//       }
//     }
//   }


//   public void lift(double clicks){
//       liftstartclicks = lift.getClicks(); // where the encoder starts
//       liftgoalclicks = liftstartclicks + clicks; // how far to go
//   }
//   public void lift(){
//     double current = lift.getClicks();
//     if(current > liftgoalclicks - 40 && current < liftgoalclicks + 40) {
//       lift.motor.setPower(0);
//       pop();
//     } else if(current < liftgoalclicks) {
//       lift.motor.setPower(0.7);
//     } else if(current > liftgoalclicks) {
//       lift.motor.setPower(-0.8);
//     }
//   }

//   public void turn() {
//     this.turning = true;
//     theta = getAngle();
//     d.setPower(0, 0, (dtheta - theta) / (Math.abs(dtheta - theta)) , 0.6);
//     if(Math.abs(theta - dtheta) < 2) { //if diff is less than 2 degrees
//       this.turning = false;
//       globala = getAngle();
//       pop();
//     }
//     // globala = 180;
//     // pop();
//   }
//   public void turn(double degrees){
//     dtheta = theta + degrees;
//   }

//   public void correct() {
//     double current = getAngle();
//     double power =  (globala - current) / (Math.abs(globala - current));
//     if (globala - current > 1) {
//       d.setPower(d.getLy(), d.getLx(), power , d.getTurbo());
//     }
//     if (globala - current < -1) {
//       d.setPower(d.getLy(), d.getLx(), power/2 , d.getTurbo());
//     }
//   }

//   public void setGlobaly () {
//     if (globala > 170 && globala < 190) {
//       globaly -= d.getClickslf();
//     } else {
//       globaly += d.getClickslf();
//     }
//     d.resetEncoderlf();
//   }

//   public void setGlobalx () {
//     if (globala > 170 && globala < 190) {
//       globalx -= d.getClickslf();
//     } else {
//       globalx += d.getClickslf();
//     }
//     d.resetEncoderlf();
//   }

//   public void forwardTo(double y){ //init forward function
//     // double dist = y-globaly;
//     if (globala > 170 && globala < 190) { //turned around
//       forward(-y);
//     } else {
//       forward(y);
//     }
//   }

//   public void forward(double clicks){
//     d.resetEncoderlf();
//     goalclicks = clicks; // how far to go
//     // relativey = globaly;
//   }
//   public void forward(){
//     double dist = Math.abs(goalclicks) - Math.abs(relativey);
//     // double p = Math.abs(dist)/120;
//     double p = 0.5; //needs to be changed back to the above line ====================================
//     if(relativey > goalclicks - 25 && relativey < goalclicks + 25) {
//       setGlobaly();
//       pop();
//     } else if(relativey < goalclicks) {
//       d.setPower(1, 0, 0, 0.6*p);
//       relativey += d.getClickslf();
//       setGlobaly();
//     } else if(relativey > goalclicks) {
//       d.setPower(-1, 0, 0, 0.6*p);
//       relativey += d.getClickslf();
//       setGlobaly();
//     }
//   }

//   public void strafeTo(double x){ //init forward function
//     if (globala > 170 && globala < 190) { //turned around
//       strafe(-x);
//     } else {
//       strafe(x);
//     }
//   }
//   public void strafe(double clicks) {
//     d.resetEncoderlf();
//     goalclicks = clicks; // how far to go
//   }
//   public void strafe() {
//     double dist = Math.abs(goalclicks) - Math.abs(relativey);
//     double p = Math.abs(dist)/120;
//     if(relativex > goalclicks - 75 && relativex < goalclicks + 75) {
//       setGlobalx();
//       pop();
//     } else if(relativex < goalclicks) {
//       d.setPower(0, -1, 0, 0.6*p);
//       relativex += d.getClickslf();
//       setGlobalx();
//     } else if(relativex > goalclicks) {
//       d.setPower(0, 1, 0, 0.6*p);
//       relativex += d.getClickslf();
//       setGlobalx();
//     }
//   }


//   public double find() {
//     double[] result = f.findSkystoneAngle();
//     double angle = result[0];
//     return angle;
//   }

//   public double getAngle() {
//     //this function taken from somewhere else

//     // We experimentally determined the Z axis is the axis we want to use for heading angle.
//     // We have to process the angle because the imu works in euler angles so the Z axis is
//     // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//     // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

//     Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//     double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

//     if (deltaAngle < -180)
//         deltaAngle += 360;
//     else if (deltaAngle > 180)
//         deltaAngle -= 360;

//     globalAngle += deltaAngle;

//     lastAngles = angles;

//     return globalAngle;
//   }

//   public void runOpMode() {

//   }

// }
