package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Constants for AdvantageKit logging mode selection.
 *
 * <p>This class determines which operating mode the robot code runs in, affecting
 * how data is logged and which hardware/simulation implementations are used.
 */
public class AdvantageConstants {

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
     *
     * <ul>
     *   <li>{@code Real}: Running on physical robot hardware with real sensors/actuators
     *   <li>{@code Sim}: Running in simulation (not currently supported)
     *   <li>{@code Replay}: Replaying from previously recorded log files for analysis
     * </ul>
     */
    public enum AdvantageMode {
        Real,
        Sim,
        Replay,
    }
}
