package org.darbots.darbotsftclib.libcore.integratedfunctions.logger;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.darbots.darbotsftclib.libcore.calculations.timecalculation.TimeCalculation;
import org.darbots.darbotsftclib.libcore.templates.log.LogContent;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

public class OpModeRunLog implements Map<String, Object> {
    public long initTime = 0;
    public String runningOpMode;
    public int threadPriority;
    public Collection<LogEntry> logEntries;
    public long startTime = -1;
    public long endTime = -1;

    public OpModeRunLog(Class runningOpMode){
        this.initTime = TimeCalculation.getCurrentMillisUTCTimeFromEPOCH();
        this.runningOpMode = runningOpMode.getName();
        this.useCurrentThreadToCalculateThreadPriority();
        this.logEntries = new ArrayList<LogEntry>();
    }

    public OpModeRunLog(Map m){
        this.logEntries = new ArrayList<LogEntry>();
        this.putAll(m);
    }

    public void useCurrentThreadToCalculateThreadPriority(){
        int threadPriority = Thread.currentThread().getPriority();
        this.threadPriority = threadPriority;
    }

    public void addLogContent(String moduleName, String logCaption, LogContent logContent, LogLevel logLevel){
        long TimeOfLog = TimeCalculation.getCurrentMillisUTCTimeFromEPOCH() - this.initTime;
        LogEntry newEntry = new LogEntry(moduleName,logCaption,TimeOfLog,logContent,logLevel);
        this.logEntries.add(newEntry);
    }

    public void startOpMode(){
        this.startTime = TimeCalculation.getCurrentMillisUTCTimeFromEPOCH() - this.startTime;
    }

    public void endOpMode(){
        this.endTime = TimeCalculation.getCurrentMillisUTCTimeFromEPOCH() - this.startTime;
    }

    @Override
    public int size() {
        return 6;
    }

    @Override
    public boolean isEmpty() {
        return false;
    }

    @Override
    public boolean containsKey(@Nullable Object key) {
        boolean expression = key.equals("startTime") || key.equals("runningOpMode") || key.equals("priority") || key.equals("logs") || key.equals("initTime") || key.equals("endTime");
        return expression;
    }

    @Override
    public boolean containsValue(@Nullable Object value) {
        Collection<Object> valueSet = this.values();
        return valueSet.contains(value);
    }

    @Nullable
    @Override
    public Object get(@Nullable Object key) {
        if(key.equals("startTime")){
            return new Long(this.startTime);
        }else if(key.equals("runningOpMode")){
            return this.runningOpMode;
        }else if(key.equals("priority")){
            return new Integer(this.threadPriority);
        }else if(key.equals("logs")) {
            return this.logEntries;
        }else if(key.equals("initTime")) {
            return new Long(this.initTime);
        }else if(key.equals("endTime")){
            return new Long(this.endTime);
        }else{
            return null;
        }
    }

    @Nullable
    @Override
    public Object put(@NonNull String key, @NonNull Object value) {
        if(key.equals("startTime") && value instanceof Number){
            this.startTime = ((Number) value).longValue();
            return new Long(this.startTime);
        }else if(key.equals("runningOpMode")){
            this.runningOpMode = value instanceof Class ? ((Class) value).getName() : value.toString();
            return this.runningOpMode;
        }else if(key.equals("priority") && value instanceof Number){
            this.threadPriority = ((Number) value).intValue();
            return new Integer(this.threadPriority);
        }else if(key.equals("logs") && value instanceof Collection) {
            Collection collectionVal = (Collection) value;
            this.logEntries.clear();
            for(Object i : collectionVal){
                if(i instanceof LogEntry){
                    this.logEntries.add((LogEntry) i);
                }else if(i instanceof Map){
                    this.logEntries.add(new LogEntry((Map) i));
                }
            }
            return this.logEntries;
        }else if(key.equals("initTime") && value instanceof Number) {
            this.initTime = ((Number) value).longValue();
            return this.initTime;
        }else if(key.equals("endTime") && value instanceof Number){
            this.endTime = ((Number) value).longValue();
            return this.endTime;
        }else{
            return null;
        }
    }

    @Nullable
    @Override
    public Object remove(@Nullable Object key) {
        return null;
    }

    @Override
    public void putAll(@NonNull Map<? extends String, ?> m) {
        for (Entry<? extends String, ?> i: m.entrySet()) {
            this.put(i.getKey(),i.getValue());
        }
    }

    @Override
    public void clear() {
        return;
    }

    @NonNull
    @Override
    public Set<String> keySet() {
        Set<String> mKeys = new TreeSet<>();
        mKeys.add("startTime");
        mKeys.add("runningOpMode");
        mKeys.add("priority");
        mKeys.add("logs");
        mKeys.add("initTime");
        mKeys.add("endTime");
        return mKeys;
    }

    @NonNull
    @Override
    public Collection<Object> values() {
        Collection<Object> mValues = new ArrayList<>();
        mValues.add(new Long(this.startTime));
        mValues.add(this.runningOpMode);
        mValues.add(new Integer(this.threadPriority));
        mValues.add(this.logEntries);
        mValues.add(new Long(this.initTime));
        mValues.add(new Long(this.endTime));
        return mValues;
    }

    @NonNull
    @Override
    public Set<Entry<String, Object>> entrySet() {
        HashSet<Entry<String,Object>> mSet = new HashSet<Entry<String, Object>>();

        mSet.add(new AbstractMap.SimpleEntry<String, Object>("startTime",new Long(this.startTime)));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("runningOpMode",this.runningOpMode));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("priority",new Integer(this.threadPriority)));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("logs",this.logEntries));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("initTime",new Long(this.initTime)));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("endTime",new Long(this.endTime)));
        return mSet;
    }
}
