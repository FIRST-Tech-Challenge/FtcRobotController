public class RobotStatus {
  
  float x;
  float y;
  float heading;
  
  public static RobotStatus(float x, float y, float heading) { // add args as needed
    this.x = x;
    this.y = y;
    this.heading = heading;
  }
  
  public float getX() {
    return this.x;
  }
  
  public float getY() {
    return this.y;
  }
  
  public float getHeading() {
    return this.heading;
  }
  
  // add methods as needed
  
}
