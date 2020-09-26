package org.firstinspires.ftc.teamcode.support.diagnostics;

import org.firstinspires.ftc.teamcode.support.events.EventManager;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A simple menu system that transforms methods with <code>@MenuEntry</code>
 *  method annotations into menu entries.
 */
public class Menu {
    private List<Entry> entries;

    public Menu() {
        entries = new ArrayList<>();
    }

    public List<Entry> getEntries() {
        return entries;
    }

    public void detectEntries(Object target) {
        for (Method method: target.getClass().getMethods()) {
            // method must be annotated with @MenuEntry
            MenuEntry annotation = method.getAnnotation(MenuEntry.class);
            if (annotation==null) continue;

            // method must have 1 or 2 arguments of EventManager type
            Class<?>[] argTypes = method.getParameterTypes();
            if (argTypes.length < 1 || argTypes.length > 2 || !argTypes[0].equals(EventManager.class)) continue;
            if (argTypes.length == 2 && !argTypes[1].equals(EventManager.class)) continue;

            // method name plus number of arguments is a unique id
            String id = method.getName() + argTypes.length;
            entries.add(new Entry(
                    id, annotation, target, method
            ));
        }
        Collections.sort(entries);
    }

    /**
     * Individual menu entry
     */
    public static final class Entry implements Comparable<Entry> {
        private String id;
        private String label;
        private String group;
        private Object target;
        private Method method;

        Entry(String id, MenuEntry annotation, Object target, Method method) {
            this.id = id;
            this.label = annotation.label();
            this.group = annotation.group();
            this.target = target;
            this.method = method;
        }

        public String getId() {
            return id;
        }

        public String getLabel() {
            return label;
        }

        public String getGroup() {
            return group;
        }

        public void invoke(EventManager gamepad1, EventManager gamepad2) throws InvocationTargetException, IllegalAccessException {
            Object[] args = new Object[method.getParameterTypes().length];
            if (args.length>0) args[0] = gamepad1;
            if (args.length==2) args[1] = gamepad2;
            method.setAccessible(true);
            method.invoke(target, args);
        }

        @Override
        public int compareTo(Entry other) {
            int result = this.group.compareTo(other.group);
            if (result==0) return this.label.compareTo(other.label);
            if (this.group.length()==0) return 1;
            if (other.group.length()==0) return -1;
            return result;
        }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (other == null || getClass() != other.getClass()) return false;
            return ((Entry) other).id.equals(id);
        }

        @Override
        public int hashCode() {
            return id.hashCode();
        }

        @Override
        public String toString() {
            return id;
        }
    }
}
