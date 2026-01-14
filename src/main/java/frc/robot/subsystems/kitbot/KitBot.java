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

    /** Constructs a KitBot subsystem with the given IO implementation */
    public KitBot(KitBotIO io) {
        this.io = io;
    }

    /** Sets the feeder motor power, [-1, 1] */
    public void setFeeder(double power) {
        io.setFeeder(power);
    }

    /** Sets the intake motor power, [-1, 1] */
    public void setIntake(double power) {
        io.setIntake(power);
    }

    /** Returns the motor data */
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
