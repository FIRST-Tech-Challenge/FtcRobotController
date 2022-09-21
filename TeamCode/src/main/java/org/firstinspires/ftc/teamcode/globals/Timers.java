package org.firstinspires.ftc.teamcode.globals;


import com.arcrobotics.ftclib.util.Timing;

public enum Timers {
    INSTANCE;

    static final private long AUTO_TIMER = 30;
    static final private long WAREHOUSE_PARKING_TIMER = 25;
    private Timing.Timer autonomousTimer;
    private Timing.Timer warehouseParkingTimer;



    public void startAutonomousTimer(){
        if (autonomousTimer != null) {
            autonomousTimer = new Timing.Timer(AUTO_TIMER);
            autonomousTimer.start();
        }

    }

    public void startWarehouseParkingTimer(){
        if (warehouseParkingTimer != null) {
            warehouseParkingTimer = new Timing.Timer(WAREHOUSE_PARKING_TIMER);
            warehouseParkingTimer.start();
        }

    }

    public Timing.Timer getAutonomousTimer(){
        return autonomousTimer;
    }
    public long getAutonomousRemainingTime(){return autonomousTimer.remainingTime();}
    public long getAutonomousElaspedTime(){return autonomousTimer.elapsedTime();}
    public boolean autonomousTimerIsDone(){return autonomousTimer.done();}

    public Timing.Timer getWarehouseParkingTimer(){
        return warehouseParkingTimer;
    }
    public long getWarehouseRemainingTime(){return warehouseParkingTimer.remainingTime();}
    public long getWarehouseElaspedTime(){return warehouseParkingTimer.elapsedTime();}
    public boolean warehouseTimerIsDone(){return warehouseParkingTimer.done();}

    public static Timers getInstance(){
        return INSTANCE;
    }
}
