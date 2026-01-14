package frc.robot.subsystems.kitbot;

/**
 * Replay implementation of {@link KitBotIO} for log playback.
 *
 * <p>This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation - all KitBot data
 * is read directly from logged {@link KitBotData}.
 *
 * <p>All methods are no-ops because all replayed data is immutable.
 */
public class KitBotReplay extends KitBotIO {

    public void setFeeder(double power) {}

    public void setIntake(double power) {}

    public void update() {}
}
