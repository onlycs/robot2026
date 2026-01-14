package frc.robot.subsystems.drivetrain.gyro;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

/**
 * Abstract base class defining the interface for gyro implementations.
 *
 * <p>Gyro implementations provide heading and angular velocity data
 * from a physical gyro sensor. Concrete implementations handle specific
 * hardware configurations (e.g., NavX, Pigeon).
 *
 * <p>The Drivetrain subsystem uses gyro implementations via dependency injection
 * to support different hardware configurations or simulation.
 */
public abstract class GyroIO {

    /** Auto-logged data structure for gyro sensor readings. */
    @AutoLog
    public static class GyroData {

        GyroData() {}

        /** Whether the gyro is connected */
        public boolean connected = false;
        /** The current gyro angle reading */
        public Angle reading = Radians.of(0);
        /** The current zero offset of the gyro */
        public Angle zero = Radians.of(0);
        /** The current gyro angular velocity */
        public AngularVelocity velocity = RadiansPerSecond.of(0);
        /** All of the raw gyro readings since the last update() call */
        public double[] readings = new double[0];

        /**
         * Calculated gyro heading
         *
         * @return the current heading adjusted for zero offset
         */
        public Angle heading() {
            return reading.minus(zero);
        }
    }

    /** The current gyro's data, since the last update() call */
    public final GyroDataAutoLogged data = new GyroDataAutoLogged();

    /** Updates this.data with the current gyro data. */
    public abstract void update();

    /** Reset gyro (device) to zero */
    protected abstract void reset();

    /**
     * Reset gyro to the specified angle
     *
     * @param zero The angle to reset the gyro to
     */
    public void reset(Rotation2d zero) {
        reset();
        data.zero = zero.getMeasure();
    }

    public final Rotation2d heading() {
        return new Rotation2d(data.heading());
    }
}
