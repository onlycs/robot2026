package frc.robot.subsystems.kitbot;

import static frc.robot.subsystems.kitbot.KitBotIO.KitBotData;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The KitBot superstructure subsystem
 *
 * <p> This class serves as a container for the KitBot subsystem.
 * It handles both the feeder and intake mechanisms.
 */
public class KitBot extends SubsystemBase {

    /** The KitBot IO implementation */
    private final KitBotIO io;

    /**
     * Constructs a KitBot subsystem with the given IO implementation
     *
     * @param io The KitBot IO implementation to use
     */
    public KitBot(KitBotIO io) {
        this.io = io;
    }

    /**
     * Sets the feeder motor power
     *
     * @param power The motor power, [-1.0, 1.0]
     */
    public void setFeeder(double power) {
        io.setFeeder(power);
    }

    /**
     * Sets the intake motor power
     *
     * @param power The motor power, [-1.0, 1.0]
     */
    public void setIntake(double power) {
        io.setIntake(power);
    }

    /**
     * KitBot motor and sensor data
     *
     * @return a copy of the current KitBot data
     */
    public KitBotData getData() {
        return io.data.clone();
    }

    /** Updates the KitBot sensor readings */
    @Override
    public void periodic() {
        io.update();
        Logger.processInputs("KitBot", io.data);
    }
}
