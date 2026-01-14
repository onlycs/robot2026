package frc.robot.subsystems.kitbot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * Abstract base class defining the interface for KitBot superstructure implementations.
 *
 * <p>The KitBot superstructure includes both the feeder and intake mechanisms.
 * This class defines the contract that all KitBot implementations must follow.
 *
 * <p>Concrete implementations (e.g., {@link KitBotSpark}) handle specific
 * hardware configurations (motor types, controllers).
 *
 * <p>The KitBot subsystem uses KitBotIO implementations via dependency injection
 * to support different hardware configurations or simulation.
 */
public abstract class KitBotIO {

    /** Auto-logged data structure for KitBot superstructure's motors. */
    @AutoLog
    public static class KitBotData {

        KitBotData() {}

        /** Feeder motor connection status (true if motor responds to commands). */
        public boolean feederConnected = false;
        /** Feeder motor supply voltage. */
        public Voltage feederVoltage = Volts.of(0);
        /** Feeder motor current draw. */
        public Current feederCurrent = Amps.of(0);

        /** Intake motor connection status (true if motor responds to commands). */
        public boolean intakeConnected = false;
        /** Intake motor supply voltage. */
        public Voltage intakeVoltage = Volts.of(0);
        /** Intake motor current draw. */
        public Current intakeCurrent = Amps.of(0);
    }

    /** KitBot subsystem data. Written to by {@link #update()}. */
    public final KitBotDataAutoLogged data = new KitBotDataAutoLogged();

    /**
     * Reads sensor data from motor controllers
     *
     * <p>This method should be called once per period to refresh
     * all sensor readings.
     */
    public abstract void update();

    /**
     * Sets the feeder motor output.
     *
     * @param power The motor power, [-1.0, 1.0]
     */
    public abstract void setFeeder(double power);

    /**
     * Sets the intake motor output.
     *
     * @param power The motor power, [-1.0, 1.0]
     */
    public abstract void setIntake(double power);
}
