package org.firstinspires.ftc.teamcode.robot.subsystems;

public class TurretLauncher {
    private LauncherRamp launcherRamp;
    private Flywheel flywheel;
    private Turret turret;
        public TurretLauncher(LauncherRamp launcherRamp, Flywheel flywheel, Turret turret) {
                this.launcherRamp = launcherRamp;
                this.flywheel = flywheel;
                this.turret = turret;
        }

        public void setTurretAngle(double angle) {
                turret.setTurretAngle(angle);
        }
        public void setHoodAngle(double angle) {
                launcherRamp.setLaunchAngle(angle);
        }

        public void setFlywheelSpeed(double speed) {
                flywheel.setTargetSpeed(speed);
        }

        public double getTurretAngle() {
                return turret.getTurretAngle();
        }
        public double getHoodAngle() {
            return launcherRamp.getCurrentAngle();
        }

        public double getFlywheelSpeed() {
            return flywheel.getCurrentSpeed();
        }

        public boolean isReadyToFire(){
            return flywheel.isAtSpeed() && launcherRamp.isAtTargetAngle() && turret.isAtAngle();
        }
}