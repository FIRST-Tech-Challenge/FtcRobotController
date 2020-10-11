package org.darbots.darbotsftclib.libcore.integratedfunctions.logger;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.darbots.darbotsftclib.libcore.templates.log.LogContent;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

public class LogEntry implements Map<String, Object> {

    public String moduleName;
    public String logCaption;
    public long timeOfLog = 0;
    public LogContent logContent;
    public LogLevel logLevel;

    public LogEntry(String moduleName, String logCaption, long timeOfLog , LogContent logContent, LogLevel logLevel){
        this.moduleName = moduleName;
        this.logCaption = logCaption;
        this.timeOfLog = timeOfLog;
        this.logContent = logContent;
        this.logLevel = logLevel;
    }

    public LogEntry(Map m){
        this.putAll(m);
    }

    @Override
    public int size() {
        return 5;
    }

    @Override
    public boolean isEmpty() {
        return false;
    }

    @Override
    public boolean containsKey(@Nullable Object key) {
        return key.equals("module") || key.equals("caption") || key.equals("timeStamp") || key.equals("content") || key.equals("level");
    }

    @Override
    public boolean containsValue(@Nullable Object value) {
        Collection<Object> valueSet = this.values();
        return valueSet.contains(value);
    }

    @Nullable
    @Override
    public Object get(@Nullable Object key) {
        if(key.equals("module")){
            return this.moduleName;
        }else if(key.equals("caption")){
            return this.logCaption;
        }else if(key.equals("timeStamp")){
            return this.timeOfLog;
        }else if(key.equals("content")){
            return this.logContent;
        }else if(key.equals("level")){
            return this.logLevel;
        }else{
            return null;
        }
    }

    @Nullable
    @Override
    public Object put(@NonNull String key, @NonNull Object value) {
        if(key.equals("module")){
            this.moduleName = value.toString();
            return this.moduleName;
        }else if(key.equals("caption")){
            this.logCaption = value.toString();
            return this.logCaption;
        }else if(key.equals("timeStamp") && value instanceof Number){
            this.timeOfLog = ((Number) value).longValue();
            return this.timeOfLog;
        }else if(key.equals("content")){
            if(value instanceof LogContent) {
                this.logContent = (LogContent) value;
            }else if(value instanceof Map){
                Map valueMap = (Map) value;
                Object valueTypeObj = valueMap.get("type");
                String valueType = valueTypeObj == null ? null : valueTypeObj.toString();
                LogContent newLogContent = LogContent.getInstance(valueType,valueMap.get("value"));
                if(newLogContent != null) {
                    this.logContent = newLogContent;
                }else{
                    return null;
                }
            }else{
                return null;
            }
            return this.logContent;
        }else if(key.equals("level")){
            if(value == null){
                return null;
            }else if(value instanceof LogLevel){
                this.logLevel = (LogLevel) value;
            }else if(value instanceof Number){
                LogLevel parsedVal = LogLevel.valueOf(((Number) value).intValue());
                if(parsedVal != null){
                    this.logLevel = parsedVal;
                }else{
                    return null;
                }
            }else{
                LogLevel parsedVal = LogLevel.valueOf(value.toString());
                if(parsedVal != null){
                    this.logLevel = parsedVal;
                }else{
                    return null;
                }
            }
            return this.logLevel;
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
        mKeys.add("module");
        mKeys.add("caption");
        mKeys.add("timeStamp");
        mKeys.add("content");
        mKeys.add("level");
        return mKeys;
    }

    @NonNull
    @Override
    public Collection<Object> values() {
        Collection<Object> mValues = new ArrayList<>();
        mValues.add(this.moduleName);
        mValues.add(this.logCaption);
        mValues.add(new Long(this.timeOfLog));
        mValues.add(this.logContent);
        mValues.add(logLevel);
        return mValues;
    }

    @NonNull
    @Override
    public Set<Entry<String, Object>> entrySet() {
        HashSet<Entry<String,Object>> mSet = new HashSet();
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("module",this.moduleName));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("caption",this.logCaption));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("timeStamp",new Long(this.timeOfLog)));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("content",this.logContent));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("level",this.logLevel));
        return mSet;
    }
}
