package frc.robot.subsystems.kitbot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public abstract class KitBotIO {

    /**
     * Data container for KitBot subsystem sensor readings and system state.
     *
     * Tracks both feeder and intake motor/encoder states. Automatically logged
     * by AdvantageKit via the @AutoLog annotation, enabling data recording
     * and replay for testing and analysis.
     */
    @AutoLog
    public static class KitBotData {

        /** Feeder motor connection status (true if motor responds to commands). */
        public boolean feederConnected = false;
        /** Feeder motor output shaft position (integrated encoder ticks). */
        public Angle feederPosition = Radians.of(0);
        /** Feeder motor current velocity. */
        public AngularVelocity feederVelocity = RadiansPerSecond.of(0);
        /** Feeder motor supply voltage. */
        public Voltage feederVoltage = Volts.of(0);
        /** Feeder motor current draw. */
        public Current feederCurrent = Amps.of(0);

        /** Intake motor connection status (true if motor responds to commands). */
        public boolean intakeConnected = false;
        /** Intake motor output shaft position (integrated encoder ticks). */
        public Angle intakePosition = Radians.of(0);
        /** Intake motor current velocity. */
        public AngularVelocity intakeVelocity = RadiansPerSecond.of(0);
        /** Intake motor supply voltage. */
        public Voltage intakeVoltage = Volts.of(0);
        /** Intake motor current draw. */
        public Current intakeCurrent = Amps.of(0);
    }

    /** KitBot subsystem sensor and state data. */
    public final KitBotDataAutoLogged data = new KitBotDataAutoLogged();

    /**
     * Reads sensor data from motor controllers
     *
     * <p>This method should be called once per period to refresh
     * all sensor readings.
     */
    public abstract void update();

    /**
     * Sets the feeder motor output. [-1.0, 1.0]
     */
    public abstract void setFeeder(double power);

    /**
     * Sets the intake motor output. [-1.0, 1.0]
     */
    public abstract void setIntake(double power);
}
