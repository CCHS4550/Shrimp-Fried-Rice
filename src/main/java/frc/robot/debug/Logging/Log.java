package frc.robot.debug.Logging;

import edu.wpi.first.wpilibj.Timer;
import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Log {
    private static final String LOGGER_NAME = Log.class.getName();
    private static final Logger LOGGER = Logger.getLogger(LOGGER_NAME);

    private Log() {}

    public static void log(String message) {
        if (LOGGER.isLoggable(Level.INFO) && hasContent(message)) {
            LOGGER.info(() -> stamp(message));
        }
    }

    public static void warn(String message) {
        if (LOGGER.isLoggable(Level.WARNING) && hasContent(message)) {
            LOGGER.warning(() -> stamp(message));
        }
    }

    public static void error(String message) {
        if (LOGGER.isLoggable(Level.SEVERE) && hasContent(message)) {
            LOGGER.severe(() -> stamp(message));
        }
    }

    private static boolean hasContent(String message) {
        return message != null && !message.isBlank();
    }

    private static String stamp(String message) {
        double timestamp = Timer.getFPGATimestamp();
        return String.format(Locale.ROOT, "[%.3f] %s", timestamp, message);
    }
}
