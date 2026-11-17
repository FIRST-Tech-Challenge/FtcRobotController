package org.firstinspires.ftc.teamcode.biobuzz;
import java.util.*;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;
/** Read-only formatting: never polls hardware or changes shooting permission. */
public final class AutoDebugSnapshot {
 private AutoDebugSnapshot(){}
 public static String pose(Sensors s){
  if(s==null||s.pose==null||!s.pose.finite())return "UNAVAILABLE";
  return String.format(Locale.US,"X %.1f in | Y %.1f in | heading %.1f deg%s",s.pose.x,s.pose.y,Math.toDegrees(s.pose.heading),s.odometryValid?"":" [INVALID - last known only]");
 }
 public static String age(Vision v,double now){
  if(v==null)return "No frame";
  double age=now-v.capturedAt;
  return !Double.isFinite(age)||age<0?"Invalid timestamp":String.format(Locale.US,"%.0f ms%s",age*1000,age>.25?" (STALE)":"");
 }
 public static String ids(Vision v,double now){
  if(v==null)return "Waiting for camera data";
  if(v.pipeline!=0)return "Pollen pipeline - AprilTags not sampled";
  if(!v.fresh(now,0))return "No fresh AprilTags";
  Set<Integer> ids=new TreeSet<Integer>();for(Target t:v.targets)if(t.id>=0)ids.add(t.id);
  return ids.isEmpty()?"No AprilTags":ids.toString();
 }
 public static String details(Vision v,double now){
  if(v==null||!v.fresh(now,0))return "No fresh tag measurements";
  StringBuilder text=new StringBuilder();int n=0;
  for(Target t:v.targets)if(t.id>=0){
   if(n++==4){text.append(" | more IDs above");break;}
   if(text.length()>0)text.append(" | ");
   text.append(String.format(Locale.US,"%d: tx %.1f, ty %.1f deg, height %s",t.id,t.tx,t.ty,Double.isFinite(t.heightInches)?String.format(Locale.US,"%.1f in",t.heightInches):"UNCALIBRATED"));
  }
  return text.length()==0?"No tag measurements":text.toString();
 }
}
