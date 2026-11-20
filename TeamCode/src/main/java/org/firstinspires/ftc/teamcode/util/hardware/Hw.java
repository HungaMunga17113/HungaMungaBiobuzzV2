package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConfig;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** hardwareMap wrapper for optional subsystems */
public final class Hw {
    private final HardwareMap map;
    private final List<String> missing = new ArrayList<>();

    public Hw(HardwareMap map) {
        this.map = map;
    }

    public <T> T get(Class<T> cls, String name) {
        T d = map.tryGet(cls, name);
        if (d == null && RobotConfig.COMPETITION) throw new IllegalStateException("missing " + name + " (RobotConfig.COMPETITION)");
        if (d == null) {
            throw new IllegalArgumentException("no " + cls.getSimpleName() + " named \"" + name
                    + "\" in the active config (push + activate on the ds?)");
        }
        return d;
    }

    /** null if missing */
    public <T> T optional(Class<T> cls, String name) {
        T d = map.tryGet(cls, name);
        if (d == null && RobotConfig.COMPETITION) throw new IllegalStateException("missing " + name + " (RobotConfig.COMPETITION)");
        if (d == null && !missing.contains(name)) missing.add(name);
        return d;
    }

    /** no-op stand-in if missing, cls has to be an interface (DcMotorEx, Servo, ...) */
    public <T> T orStub(Class<T> cls, String name) {
        T d = optional(cls, name);
        return d != null ? d : stub(cls);
    }

    public List<String> missing() {
        return Collections.unmodifiableList(missing);
    }

    public static boolean isStub(Object device) {
        return device != null && Proxy.isProxyClass(device.getClass())
                && Proxy.getInvocationHandler(device) == STUB;
    }

    public static <T> T stub(Class<T> cls) {
        if (!cls.isInterface()) throw new IllegalArgumentException(cls.getSimpleName() + " isn't an interface, use optional()");
        return cls.cast(Proxy.newProxyInstance(cls.getClassLoader(), new Class<?>[]{cls}, STUB));
    }

    private static final InvocationHandler STUB = (proxy, method, args) -> {
        switch (method.getName()) {
            case "equals": return proxy == args[0];
            case "hashCode": return System.identityHashCode(proxy);
            case "toString": return "stub";
            default: return defaultFor(method.getReturnType());
        }
    };

    private static Object defaultFor(Class<?> t) {
        if (t == boolean.class) return false;
        if (t == int.class) return 0;
        if (t == double.class) return 0.0;
        if (t == long.class) return 0L;
        if (t == float.class) return 0f;
        if (t == short.class) return (short) 0;
        if (t == byte.class) return (byte) 0;
        if (t == char.class) return '\0';
        if (t == String.class) return "stub";
        if (t.isEnum()) return t.getEnumConstants()[0];
        return null;
    }
}
