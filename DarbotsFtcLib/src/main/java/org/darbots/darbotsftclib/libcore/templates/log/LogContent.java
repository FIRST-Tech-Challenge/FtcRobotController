/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

package org.darbots.darbotsftclib.libcore.templates.log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.logContents.Number_Log;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.logContents.String_Log;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

public abstract class LogContent implements Map<String, Object> {
    public abstract LogType getContentType();
    public abstract Object getContentValue();
    public abstract void setContentValue(Object contentVal);

    @Override
    public int size() {
        return 2;
    }

    @Override
    public boolean isEmpty() {
        return false;
    }

    @Override
    public boolean containsKey(@Nullable Object key) {
        boolean expression = key.equals("type") || key.equals("value");
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
        if(key.equals("type")){
            return this.getContentType().name();
        } else if (key.equals("value")) {
            return this.getContentValue();
        }
        return null;
    }

    @Nullable
    @Override
    public Object put(@NonNull String key, @NonNull Object value) {
        if(key.equals("value")){
            this.setContentValue(value);
        }
        return null;
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
        mKeys.add("type");
        mKeys.add("value");
        return mKeys;
    }

    @NonNull
    @Override
    public Collection<Object> values() {
        Collection<Object> mValues = new ArrayList<>();
        mValues.add(this.getContentType().name());
        mValues.add(this.getContentValue());
        return mValues;
    }

    @NonNull
    @Override
    public Set<Entry<String, Object>> entrySet() {
        HashSet<Entry<String,Object>> mSet = new HashSet<>();
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("type", this.getContentType().name()));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("value", this.getContentValue()));
        return mSet;
    }

    @Nullable
    public static LogContent getInstance(String logType, Object value){
        if(logType.equals(LogType.STRING_LOG.name())) {
            //Initialize a String Log
            return new String_Log(value);
        }else if(logType.equals(LogType.NUMBER_LOG.name())){
            return new Number_Log(value);
        }else{
            return null;
        }
    }
}
