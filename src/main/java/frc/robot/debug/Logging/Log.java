package frc.robot.debug.Logging;

import edu.wpi.first.wpilibj.Timer;
import java.util.Locale;
import java.util.logging.Logger;

public class Log {
    private static final Logger LOGGER = Logger.getLogger(Log.class.getName());

    private Log() {}

    public static void log(String message) {
        if (hasContent(message)) {
            LOGGER.info(stamp(message));
        }
    }

    public static void warn(String message) {
        if (hasContent(message)) {
            LOGGER.warning(stamp(message));
        }
    }

    public static void error(String message) {
        if (hasContent(message)) {
            LOGGER.severe(stamp(message));
        }
    }

    private static boolean hasContent(String message) {
        return message != null && !message.isBlank();
    }

    private static String stamp(String message) {
        return String.format(Locale.ROOT, "[%.3f] %s", Timer.getFPGATimestamp(), message);
    }
}
