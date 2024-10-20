package org.firstinspires.ftc.teamcode.core;

import java.util.HashMap;

public class EventTracker {
    // HashMap to store event names and their corresponding timestamps
    private HashMap<String, Double> eventTimestamps;

    // Constructor
    public EventTracker() {
        eventTimestamps = new HashMap<>();
    }

    public boolean doEvent(String eventName, double currentTime, double waitTime)
    {
        if(currentTime > getLastTimestamp(eventName))
        { //  allowed to do event
            setTimestamp(eventName, currentTime + waitTime);
            return true;
        }
        else
        { // not allowed to do event
            return false;
        }

    }
    // Method to set the timestamp for a specific event
    public void setTimestamp(String eventName, double timestamp) {
        eventTimestamps.put(eventName, timestamp);
    }

    // Method to get the last timestamp of a specific event
    public double getLastTimestamp(String eventName) {
        double timeStamp = eventTimestamps.getOrDefault(eventName, 0.0);
        return timeStamp;
    }

    // Optional: Method to check if an event has been recorded
    public boolean hasEvent(String eventName) {
        return eventTimestamps.containsKey(eventName);
    }

    // Optional: Method to clear all timestamps
    public void clearTimestamps() {
        eventTimestamps.clear();
    }
}