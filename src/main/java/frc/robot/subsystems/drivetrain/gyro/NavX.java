package frc.robot.subsystems.drivetrain.gyro;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.ControlConstants.kGyroFactor;

import com.studica.frc.AHRS;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.constants.IOConstants;
import frc.robot.util.Alerter;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

/**
 * NavX gyro implementation of GyroIO.
 *
 * <p>Interfaces with the NavX gyro sensor to provide heading and angular velocity data.
 * Uses a separate thread to read gyro data at a fixed rate for improved responsiveness.
 */
public class NavX extends GyroIO {

    /** The NavX gyro sensor instance */
    final AHRS gyro;

    /** Threaded notifier for periodic gyro reads */
    final Notifier thread = new Notifier(this::read);
    /** Queue to store gyro readings */
    final Queue<Double> readings = new ArrayBlockingQueue<>(20);
    /** Queue lock to prevent multiple concurrent reads/writes */
    final ReentrantLock lock = new ReentrantLock();
    /** Gyro read rate in Hz */
    final double rate = 100;

    /** Constructs a NavX gyro IO implementation. */
    public NavX() {
        this.gyro = new AHRS(IOConstants.Drivetrain.kGyroPort, (byte) rate);
        this.gyro.enableLogging(true);

        Alerter.getInstance().register(this.gyro);
        thread.setName("GyroSensor");
        thread.startPeriodic(1.0 / rate);
    }

    /** Reads gyro data and stores it in the readings queue */
    void read() {
        if (!lock.tryLock()) return; // if we can't get the lock, don't read the gyro
        this.readings.add(measure().in(Radians));
        lock.unlock(); // release the lock
    }

    /**
     * Measure the current gyro angle
     * @return The current gyro angle
     */
    Angle measure() {
        return Degrees.of(gyro.getAngle() * kGyroFactor);
    }

    /**
     * Measure the current gyro rate
     * @return The current gyro angular velocity
     */
    AngularVelocity rate() {
        return DegreesPerSecond.of(gyro.getRate() * kGyroFactor);
    }

    /**
     * Collect all queued gyro readings into an array
     * @return Array of gyro readings since last update
     */
    double[] collect() {
        return readings.stream().mapToDouble(Double::doubleValue).toArray();
    }

    @Override
    public void update() {
        lock.lock(); // prevent threaded writes to data during our reads.

        data.connected = gyro.isConnected();
        data.reading = this.measure();
        data.velocity = this.rate();
        data.readings = this.collect();
        readings.clear();

        lock.unlock(); // release the lock
    }

    @Override
    public void reset() {
        gyro.reset();
    }
}
