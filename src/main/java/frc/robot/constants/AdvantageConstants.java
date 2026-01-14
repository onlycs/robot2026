package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Constants for AdvantageKit logging mode selection.
 *
 * <p>This class determines which operating mode the robot code runs in, affecting
 * how data is logged and which hardware/simulation implementations are used.
 */
public class AdvantageConstants {

    private AdvantageConstants() {}

    /**
     * The current robot operating mode.
     *
     * <p>Automatically detected based on the robot environment:
     * <ul>
     *   <li>{@link AdvantageMode#Real}: Running on actual robot hardware
     *   <li>{@link AdvantageMode#Replay}: Running in a replay session from log files
     * </ul>
     */
    public static final AdvantageMode kCurrentMode = RobotBase.isReal()
        ? AdvantageMode.Real
        : AdvantageMode.Replay;

    /**
     * Robot operating modes for AdvantageKit.
     */
    public enum AdvantageMode {
        /** Running on physical robot hardware with real sensors/actuators */
        Real,
        /** Running in simulation (not currently supported) */
        Sim,
        /** Replaying from previously recorded log files for analysis */
        Replay,
    }
}
