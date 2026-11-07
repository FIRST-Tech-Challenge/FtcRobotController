package org.firstinspires.ftc.teamcode.robot.subsystems;

public class TurretLauncher {
    private LauncherRamp launcherRamp;
    //TODO: need to make flywheel class
    //private Flywheel flywheel;
    private Turret turret;
    //TODO: Actual fuction to use once Flywheel done
//        public TurretLauncher(LauncherRamp launcherRamp, Flywheel flywheel, Turret turret) {
        public TurretLauncher(LauncherRamp launcherRamp, Turret turret) {
                this.launcherRamp = launcherRamp;
                //TODO: Uncomment once flywheel class made
//                this.flywheel = flywheel;
                this.turret = turret;
        }

        public void setTurretAngle(double angle) {
                turret.setTurretAngle(angle);
        }
        public void setHoodAngle(double angle) {
                launcherRamp.setLaunchAngle(angle);
        }
    //TODO: need to add flywheel class

        public void setFlywheelSpeed(double speed) {
//                flywheel.setTargetSpeed(speed);
        }

        public double getTurretAngle() {
                return turret.getTurretAngle();
        }
        public double getHoodAngle() {
            return launcherRamp.getCurrentAngle();
        }
    //TODO: need flywheel class

        public double getFlywheelSpeed() {
//            return flywheel.getCurrentSpeed();
            return 0;
        }

    //TODO: add the functions to flywheel when added
        public boolean isReadyToFire(){
            //TODO: Use commented out code once flywheel implemented
//            return flywheel.isAtTargetSpeed() && launcherRamp.isAtTargetAngle() && turret.isAtTargetAngle();
            return launcherRamp.isAtTargetAngle() && turret.isAtAngle();
        }
}