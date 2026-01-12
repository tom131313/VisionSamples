package frc.robot.utils;

import frc.robot.RobotContainer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.stream.Collectors;

/**
 * Select command stages to log and where to log them
 */
public class CommandSchedulerLog {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    public enum CommandStageSelector {
        initialize, interrupt, finish, execute
    }

    public enum LogsSelector {
        useConsole, useDataLog, useShuffleBoardLog
    };

    private final HashMap<String, Integer> currentCommands = new HashMap<String, Integer>();
    private final NetworkTable nt;
    private final StringEntry initializeCommandLogEntry;
    private final StringEntry interruptCommandLogEntry;
    private final StringEntry finishCommandLogEntry;
    private final StringEntry executeCommandLogEntry;
    private EnumSet<LogsSelector> logsSelector;

    /**
     * Command Event Loggers
     * <p>
     * Set the command scheduler to log events for command initialize, interrupt, finish, and execute.
     * <p>
     * Log to the Console/Terminal, ShuffleBoard or the WPILib DataLog tool.
     * <p>
     * If ShuffleBoard is recording (start it manually), these events are added to the recording.
     * Convert recording to csv and they show nicely in Excel.
     * <p>
     * Caution that {@link CommandSchedulerLog} assumes its logging is turned on and not suppressed
     * by interactions with {@link RobotContainer#configDataLog()}.
     * If using DataLog tool, the recording is via NT so tell NT to send EVERYTHING to the DataLog
     * (or at least the CommandLog table entries produced by this logging).
     * Run DataLog tool to retrieve log from roboRIO and convert the log to a csv table that may be
     * viewed nicely in Excel.
     * <p>
     * Note the comment in execute logging that only the first execute is logged unless changed.
     * <p>
     * Note that use of the DataLog creates SmartDashboard/ShuffleBoard entries but are not the same
     * as use of the ShuffleBoardLog. The ShuffleBoardLog has event markers independent of the DataLog.
     * <p>
     * Select command stages to log - typically allOf() or noneOf() (or individually)
     * <p>
     * Select 1 or more individual Command logging methods
     * <p>
     * or none EnumSet.noneOf(LogsSelector.class)
     * <p>
     * or all; less typing but obscures selections in EnumSet.allOf(LogsSelector.class)
     * <p>
     * Log Command Scheduler actions for command initialize, execute, interrupt, finish
     * <p>
     * This class could be used this way, for example:
     * <pre>
     * <code>
     *   new CommandSchedulerLog(EnumSet.allOf(CommandStageSelector.class),
     *       EnumSet.of(LogsSelector.useConsole, LogsSelector.useDataLog));
     * </code>
     * </pre>
     * @param commandStageSelector
     * @param logsSelector
     */
    public CommandSchedulerLog(EnumSet<CommandStageSelector> commandStageSelector, EnumSet<LogsSelector> logsSelector) {
        this.logsSelector = logsSelector;

        // DataLog via NT so establish NT and the connection to DataLog
        if (logsSelector.contains(LogsSelector.useDataLog)) {
            DataLogManager.logNetworkTables(true); // CAUTION - this puts all NT to the DataLog
        }

        final String networkTableName = "CommandLog";
        nt = NetworkTableInstance.getDefault().getTable(networkTableName);

        if (logsSelector.contains(LogsSelector.useDataLog)) {
            initializeCommandLogEntry = nt.getStringTopic("Commands/initialize").getEntry("");
            interruptCommandLogEntry = nt.getStringTopic("Commands/interrupt").getEntry("");
            finishCommandLogEntry = nt.getStringTopic("Commands/finish").getEntry("");
            executeCommandLogEntry = nt.getStringTopic("Commands/execute").getEntry("");
        } else {
            initializeCommandLogEntry = null;
            interruptCommandLogEntry = null;
            finishCommandLogEntry = null;
            executeCommandLogEntry = null;
        }

        if (commandStageSelector.contains(CommandStageSelector.initialize)) {
            logCommandInitialize();
        }
        if (commandStageSelector.contains(CommandStageSelector.interrupt)) {
            logCommandInterrupt();
        }
        if (commandStageSelector.contains(CommandStageSelector.finish)) {
            logCommandFinish();
        }
        if (commandStageSelector.contains(CommandStageSelector.execute)) {
            logCommandExecute(); // Can (optionally) generate a lot of output
        }
    }

    /**
     * Set callback to log commands that run the initialize method.
     */
    public void logCommandInitialize() {
        CommandScheduler.getInstance().onCommandInitialize(
                (command) -> {
                    String key = command.getClass().getSimpleName() + "/" + command.getName();
                    String requirements = command.getRequirements().stream()
                            .map(subsystem -> subsystem.getClass().getSimpleName())
                            .collect(Collectors.joining(", ", "{", "}"));

                    if (logsSelector.contains(LogsSelector.useConsole)) {
                        System.out.println("Command initialized : " + key + " " + requirements);
                    }
                    if (logsSelector.contains(LogsSelector.useDataLog)) {
                        initializeCommandLogEntry.set(key + " " + requirements);
                    }
                    if (logsSelector.contains(LogsSelector.useShuffleBoardLog)) {
                        Shuffleboard.addEventMarker("Command initialized",
                                key + " " + requirements, EventImportance.kNormal);
                    }

                    currentCommands.put(key, 0);
                });
    }

    /**
     * Set callback to log commands that have been interrupted.
     */
    public void logCommandInterrupt() {
        CommandScheduler.getInstance().onCommandInterrupt(
                (command, interruptedBy) -> {
                    String interrupter;
                    if (interruptedBy.isPresent()) {
                        interrupter = "interrupted by command " + command.getClass().getSimpleName() + "/"
                                + interruptedBy.get().getName();
                    } else {
                        interrupter = "interrupted"; // interrupted not by a command - mode change, cancelled, timeOut,
                                                     // until, etc.
                    }

                    String key = command.getClass().getSimpleName() + "/" + command.getName();
                    String runs = " after " + currentCommands.getOrDefault(key, 0) + " runs " + interrupter;

                    if (logsSelector.contains(LogsSelector.useConsole)) {
                        System.out.println(key + runs);
                    }
                    if (logsSelector.contains(LogsSelector.useDataLog)) {
                        interruptCommandLogEntry.set(key + runs);
                    }
                    if (logsSelector.contains(LogsSelector.useShuffleBoardLog)) {
                        Shuffleboard.addEventMarker("Command interrupted", key + runs, EventImportance.kNormal);
                    }

                    currentCommands.put(key, 0);
                });
    }

    /**
     * Set callback to log commands that run the finish method.
     */
    public void logCommandFinish() {
        CommandScheduler.getInstance().onCommandFinish(
                (command) -> {
                    String key = command.getClass().getSimpleName() + "/" + command.getName();
                    String runs = " after " + currentCommands.getOrDefault(key, 0) + " runs";

                    if (logsSelector.contains(LogsSelector.useConsole)) {
                        System.out.println("Command finished : " + key + runs);
                    }
                    if (logsSelector.contains(LogsSelector.useDataLog)) {
                        finishCommandLogEntry.set(key + runs);
                    }
                    if (logsSelector.contains(LogsSelector.useShuffleBoardLog)) {
                        Shuffleboard.addEventMarker("Command finished", key, EventImportance.kNormal);
                    }

                    currentCommands.put(key, 0);
                });
    }

    /**
     * Set callback to log commands that run the execute() method.
     * 
     * <p>
     * This can generate a lot of events so logging is suppressed except for the first
     * occurrence of execute(). Total count of execute() is logged at command end.
     * 
     * <p>
     * Recompile without the if/else to get all execute() logged.
     */
    public void logCommandExecute() {
        CommandScheduler.getInstance().onCommandExecute(
                (command) -> {
                    String key = command.getClass().getSimpleName() + "/" + command.getName();

                    if (currentCommands.getOrDefault(key, 0) == 0) // suppress all but first execute
                    {
                        if (logsSelector.contains(LogsSelector.useConsole)) {
                            System.out.println("Command executed : " + key);
                        }
                        if (logsSelector.contains(LogsSelector.useDataLog)) {
                            executeCommandLogEntry.set(key);
                        }
                        if (logsSelector.contains(LogsSelector.useShuffleBoardLog)) {
                            Shuffleboard.addEventMarker("Command executed", key, EventImportance.kNormal);
                        }

                        currentCommands.put(key, 1); // first time through count is 1
                    } else {
                        // Increment total count to log when the command ends.
                        currentCommands.put(key, currentCommands.get(key) + 1);
                    }
                });
    }
}
