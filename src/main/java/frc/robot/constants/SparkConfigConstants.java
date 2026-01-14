package frc.robot.constants;

import static frc.robot.util.MathUtil.kTau;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * REV Spark motor controller configuration presets.
 *
 * <p>Defines complete {@link SparkMaxConfig} objects for each motor type on the robot.
 * Configurations include:
 * <ul>
 *   <li>Current limits (prevent motor damage and brownouts)
 *   <li>Encoder conversion factors (position and velocity)
 *   <li>Voltage compensation (consistent behavior across battery voltages)
 *   <li>PID gains and feedforward coefficients
 *   <li>Output limits and wrapping behavior
 *   <li>Idle modes (brake vs. coast)
 * </ul>
 *
 * <p>Configurations are applied to motors via {@link com.revrobotics.spark.SparkBase#configure}
 * during subsystem initialization.
 */
public class SparkConfigConstants {

    /**
     * Reset mode for configuration apply operations.
     *
     * <p>{@code kResetSafeParameters} resets all parameters to factory defaults
     * before applying new configuration, ensuring no stale settings remain.
     */
    public static final ResetMode kResetMode = ResetMode.kResetSafeParameters;

    /**
     * Persistence mode for configuration apply operations.
     *
     * <p>{@code kPersistParameters} saves configuration to Spark flash memory,
     * so settings survive power cycles and reduce CAN traffic on boot.
     */
    public static final PersistMode kPersistMode =
        PersistMode.kPersistParameters;

    /**
     * Swerve drivetrain motor configurations.
     *
     * <p>Separate configs for drive (NEO) and turn (NEO 550) motors.
     */
    public static final class Drivetrain {

        /** Complete configuration for swerve drive motors (velocity control). */
        public static final SparkMaxConfig kDrive;

        /** Complete configuration for swerve turn motors (position control). */
        public static final SparkMaxConfig kTurn;

        static {
            kDrive = new SparkMaxConfig();
            kTurn = new SparkMaxConfig();

            // ===== CURRENT LIMITS =====
            // Prevent motor damage and battery voltage sag
            kDrive.smartCurrentLimit((int) MotorConstants.Neo.kCurrentLimit);
            kTurn.smartCurrentLimit((int) MotorConstants.Neo550.kCurrentLimit);

            // ===== VOLTAGE COMPENSATION =====
            // Normalize motor output for battery voltage 11.0-13.0V
            kDrive.voltageCompensation(MotorConstants.kNominalVoltage);
            kTurn.voltageCompensation(MotorConstants.kNominalVoltage);

            // ===== ENCODER CONVERSIONS =====
            // Drive motor: NEO internal encoder
            kDrive.encoder.positionConversionFactor(
                SwerveConstants.DriveEncoder.kPositionFactor
            );
            kDrive.encoder.velocityConversionFactor(
                SwerveConstants.DriveEncoder.kVelocityFactor
            );

            // Turn motor: Absolute encoder (through-bore)
            kTurn.absoluteEncoder.positionConversionFactor(
                SwerveConstants.TurnEncoder.kPositionFactor
            );
            kTurn.absoluteEncoder.velocityConversionFactor(
                SwerveConstants.TurnEncoder.kVelocityFactor
            );

            // ===== ENCODER UVW SETTINGS =====
            kDrive.encoder.uvwMeasurementPeriod(MotorConstants.Neo.kUvwPeriod);
            kDrive.encoder.uvwAverageDepth(MotorConstants.Neo.kUvwDepth);
            kTurn.absoluteEncoder.averageDepth(MotorConstants.Neo550.kUvwDepth);

            // ===== TURN ABSOLUTE ENCODER SETUP =====
            kTurn.absoluteEncoder.inverted(
                SwerveConstants.TurnEncoder.kInverted
            );
            kTurn.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

            // ===== TURN PID WRAPPING =====
            // Enable continuous input (0 to 2π wraps around)
            kTurn.closedLoop.positionWrappingEnabled(true);
            kTurn.closedLoop.positionWrappingMinInput(
                ControlConstants.DrivetrainTurn.kMinInput
            );
            kTurn.closedLoop.positionWrappingMaxInput(
                ControlConstants.DrivetrainTurn.kMaxInput
            );

            // ===== DRIVE PID + FEEDFORWARD =====
            kDrive.closedLoop.pid(
                ControlConstants.DrivetrainDrive.kP,
                ControlConstants.DrivetrainDrive.kI,
                ControlConstants.DrivetrainDrive.kD
            );
            kDrive.closedLoop.feedForward.sva(
                ControlConstants.DrivetrainDrive.kS,
                ControlConstants.DrivetrainDrive.kV,
                ControlConstants.DrivetrainDrive.kA
            );
            kDrive.closedLoop.outputRange(
                ControlConstants.DrivetrainDrive.kMin,
                ControlConstants.DrivetrainDrive.kMax
            );

            // ===== TURN PID =====
            kTurn.closedLoop.pid(
                ControlConstants.DrivetrainTurn.kP,
                ControlConstants.DrivetrainTurn.kI,
                ControlConstants.DrivetrainTurn.kD
            );
            kTurn.closedLoop.outputRange(
                ControlConstants.DrivetrainTurn.kMinOutput,
                ControlConstants.DrivetrainTurn.kMaxOutput
            );

            // ===== IDLE MODES =====
            // Brake mode: resist motion when disabled (better control)
            kDrive.idleMode(SwerveConstants.DriveMotor.kIdleMode);
            kTurn.idleMode(SwerveConstants.TurnMotor.kIdleMode);
        }
    }

    /**
     * KitBot motor configurations.
     *
     * <p>Separate configs for feeder and intake motors.
     */
    public static final class KitBot {

        /** Complete configuration for feeder motor. */
        public static final SparkMaxConfig kFeeder;

        /** Complete configuration for intake motor. */
        public static final SparkMaxConfig kIntake;

        static {
            kFeeder = new SparkMaxConfig();
            kIntake = new SparkMaxConfig();

            // ===== CURRENT LIMITS =====
            // Prevent motor damage and battery voltage sag
            kFeeder.smartCurrentLimit(30);
            kIntake.smartCurrentLimit(30);

            // ===== VOLTAGE COMPENSATION =====
            // Normalize motor output for battery voltage 11.0-13.0V
            kFeeder.voltageCompensation(MotorConstants.kNominalVoltage);
            kIntake.voltageCompensation(MotorConstants.kNominalVoltage);

            // ===== ENCODER CONVERSIONS =====
            // Convert motor outputs from RPM and revs to rad/s and radians
            kFeeder.encoder.positionConversionFactor(kTau);
            kFeeder.encoder.velocityConversionFactor(kTau / 60.0);
            kIntake.encoder.positionConversionFactor(kTau);
            kIntake.encoder.velocityConversionFactor(kTau / 60.0);

            // ===== UVW SETTINGS =====
            kFeeder.encoder.uvwMeasurementPeriod(MotorConstants.Neo.kUvwPeriod);
            kFeeder.encoder.uvwAverageDepth(MotorConstants.Neo.kUvwDepth);
            kIntake.encoder.uvwMeasurementPeriod(MotorConstants.Neo.kUvwPeriod);
            kIntake.encoder.uvwAverageDepth(MotorConstants.Neo.kUvwDepth);

            // ===== VOLTAGE COMPENSATION =====
            // Normalize motor output for battery voltage 11.0-13.0V
            kFeeder.voltageCompensation(MotorConstants.kNominalVoltage);
            kIntake.voltageCompensation(MotorConstants.kNominalVoltage);

            // ===== IDLE MODES =====
            // Brake mode: resist motion when disabled (better control)
            kFeeder.idleMode(IdleMode.kBrake);
            kIntake.idleMode(IdleMode.kBrake);
        }
    }
}
