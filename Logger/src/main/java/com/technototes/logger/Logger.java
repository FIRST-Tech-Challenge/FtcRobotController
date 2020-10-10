package com.technototes.logger;

import com.technototes.logger.entry.BooleanEntry;
import com.technototes.logger.entry.Entry;
import com.technototes.logger.entry.NumberBarEntry;
import com.technototes.logger.entry.NumberEntry;
import com.technototes.logger.entry.NumberSliderEntry;
import com.technototes.logger.entry.StringEntry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;


public class Logger {

    public Entry[] entries;
    public ArrayList<Entry> unindexedEntries;
    public Telemetry telemetry;
    public Object root;
    public int total = 0, max = -1;

    public Logger(Telemetry tel, Object r) {
        root = r;
        telemetry = tel;
        entries = new Entry[30];
        unindexedEntries = new ArrayList<>();
        configure(r);

        mergeEntries();

    }

    private void mergeEntries() {
        for(int i = 0; unindexedEntries.size() > 0; i++){
            if(entries[i] == null) {
                entries[i] = unindexedEntries.remove(0);
            }
        }
        entries = Arrays.copyOfRange(entries, 0, Math.max(total, max+1));

    }

    public void update() {
        for(int i = 0; i < entries.length; i++){
            telemetry.addLine((i > 9 ? i+"| " : i+" | ")+ (entries[i] == null ? "" : entries[i].toString()));
        }
    }

    public void configure(Object root) {
        for (Field field : root.getClass().getDeclaredFields()) {
            try {
                Object o = field.get(root);
                if (o instanceof Loggable) {
                    configure(o);
                } else if (field.isAnnotationPresent(Log.class) || field.isAnnotationPresent(Log.Number.class) ||
                        field.isAnnotationPresent(Log.NumberSlider.class) || field.isAnnotationPresent(Log.NumberBar.class)) {
                    if(field.getClass().isPrimitive()){
                        set(field.getDeclaredAnnotations(), field, root);
                    }else {
                        for (Method m : o.getClass().getDeclaredMethods()) {
                            if (m.isAnnotationPresent(Log.class)) {
                                set(field.getDeclaredAnnotations(), m, o);
                            }

                        }
                    }
                }
            } catch (IllegalAccessException e) {
                continue;
            }
        }
        for (Method m : root.getClass().getDeclaredMethods()) {
            set(m.getDeclaredAnnotations(), m, root);
        }

    }
    public void set(Annotation[] a, Method m, Object root){
        set(a, () -> {
            try {
                return m.invoke(root);
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            } catch (InvocationTargetException e) {
                e.printStackTrace();
            }
            return null;
        });
    }
    public void set(Annotation[] a, Field m, Object root){
        set(a, () -> {
            try {
                return m.get(root);
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
            return null;
        });
    }
    public void set(Annotation[] a, Supplier<?> m) {
        Entry e = null;
        for(Annotation as : a){
            if(as instanceof Log.NumberSlider){
                e = new NumberSliderEntry(((Log.NumberSlider) as).name(), (Supplier<Number>)m,
                        ((Log.NumberSlider) as).index(), ((Log.NumberSlider) as).min(),
                        ((Log.NumberSlider) as).max(), ((Log.NumberSlider) as).scale());
                processEntry(e);
                return;
            } else if(as instanceof Log.NumberBar){
                e = new NumberBarEntry(((Log.NumberBar) as).name(), (Supplier<Number>)m,
                        ((Log.NumberBar) as).index(), ((Log.NumberBar) as).min(),
                        ((Log.NumberBar) as).max(), ((Log.NumberBar) as).scale());
                processEntry(e);
                return;
            } else if (as instanceof Log.Number){
                e = new NumberEntry(((Log.Number) as).name(), (Supplier<Number>)m,
                        ((Log.Number) as).index());
                processEntry(e);
                return;
            } else if (as instanceof Log){
                e = new StringEntry(((Log) as).name(), (Supplier<String>)m,
                        ((Log) as).index());
                processEntry(e);
                return;
            } else if (as instanceof Log.Boolean){
                e = new BooleanEntry(((Log.Boolean) as).name(), (Supplier<Boolean>) m, ((Log.Boolean) as).index(),
                        ((Log.Boolean) as).valueWhenTrue(), ((Log.Boolean) as).valueWhenFalse());
                processEntry(e);
                return;
            }
        }
    }
    private Object invoke(Method m, Object r){
        m.setAccessible(true);
        try {
            return m.invoke(r);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (InvocationTargetException e) {
            e.printStackTrace();
        }
        return null;
    }

    private void processEntry(Entry e){
        if(e.x != -1) {
            if(entries[e.x] != null){
                unindexedEntries.add(e);
            }   else{
                entries[e.x] = e;
            }
        }else{
            unindexedEntries.add(e);
        }
        total++;
        max = Math.max(max, e.x);
    }

}
