package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.util.MathPlus.kTau;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import java.util.Arrays;

/**
 * Swerve drive configuration constants.
 *
 * <p>Contains all parameters required to configure a 4-module swerve drivetrain:
 * <ul>
 *   <li>Module positions and CAN IDs
 *   <li>Motor gear ratios and inversions
 *   <li>Encoder conversion factors
 *   <li>Wheel specifications (radius, friction coefficient)
 *   <li>Maximum speeds (linear and angular)
 *   <li>SwerveDriveKinematics for odometry and control
 * </ul>
 *
 * <p>All gear ratios follow the convention {@code kReduction motorRotation : 1 wheelRotations}.
 */
public final class SwerveConstants {

    private SwerveConstants() {}

    /** Swerve drive kinematics object for odometry and chassis speed conversion. */
    public static final SwerveDriveKinematics kKinematics =
        new SwerveDriveKinematics(
            Arrays.stream(ModuleId.values())
                .map(ModuleId::translation)
                .toArray(Translation2d[]::new)
        );

    /**
     * Swerve module hardware configuration.
     *
     * <p>Each module is assigned:
     * <ul>
     *   <li>A unique index value (1-4) for CAN ID calculation
     *   <li>Driving motor ID, calculated as {@code index * 10}
     *   <li>Turning motor ID, calculated as {@code driveId + 5}
     *   <li>Absolute encoder angular offset, converting from robot-centric to wheel-centric angles
     *   <li>Position relative to robot center (X-forward, Y-left)
     * </ul>
     *
     * <p><strong>Module ordering:</strong> Enum variants are declared in the following
     * order to ensure that the <code>values()</code> method:
     * <p><code>[frontLeft, frontRight, rearLeft, rearRight]</code>
     *
     * Should be FrontLeft=1x and go clockwise from top-down view.
     */
    public enum ModuleId {
        /** Front-left swerve module. */
        kFrontLeft(1),
        /** Front-right swerve module. */
        kFrontRight(2),
        /** Rear-left swerve module. */
        kRearLeft(3),
        /** Rear-right swerve module. */
        kRearRight(4);

        /** The module's index value (1-4). */
        public final int value;

        ModuleId(int i) {
            this.value = i;
        }

        /**
         * Drive motor CAN ID
         *
         * @return The CAN ID of the driving motor for this module
         */
        public int driveId() {
            return this.value * 10;
        }

        /**
         * Turn motor CAN ID
         *
         * @return The CAN ID of the turning motor for this module
         */
        public int turnId() {
            return this.driveId() + 5;
        }

        /**
         * Absolute encoder angular offset
         *
         * @return The angular offset of this module's absolute encoder
         */
        public double angularOffset() {
            return switch (this) {
                case kFrontLeft -> -Math.PI / 2;
                case kFrontRight -> 0;
                case kRearLeft -> Math.PI;
                case kRearRight -> Math.PI / 2;
            };
        }

        /**
         * Translation from robot center to module.
         *
         * @return The translation of this module relative to robot center
         */
        public Translation2d translation() {
            double absX = RobotConstants.DriveBase.kWheelBase / 2;
            double absY = RobotConstants.DriveBase.kTrackWidth / 2;

            return switch (this) {
                case kFrontLeft -> new Translation2d(absX, absY);
                case kFrontRight -> new Translation2d(absX, -absY);
                case kRearLeft -> new Translation2d(-absX, absY);
                case kRearRight -> new Translation2d(-absX, -absY);
            };
        }
    }

    /**
     * Drive motor (NEO) configuration.
     *
     * <p>MAXSwerve module uses a multi-stage gearing system:
     * <ul>
     *   <li>Pinion gear (14T) on motor shaft
     *   <li>Bevel gear (45T) and bevel pinion (15T) for 90° turn
     *   <li>Spur gear (22T) final reduction to wheel
     * </ul>
     */
    public static final class DriveMotor {

        private DriveMotor() {}

        /**
         * Number of teeth on the pinion gear.
         *
         * <p>According to REV docs, can be 12T, 13T, or 14T.
         * This robot uses 14T for higher top speed.
         */
        public static final double kPinionTeeth = 14.0;

        /** Number of teeth on the wheel's bevel gear (45T). */
        public static final double kBevelGearTeeth = 45.0;

        /** Number of teeth on the first-stage spur gear (22T). */
        public static final double kSpurTeeth = 22.0;

        /** Number of teeth on the bevel pinion (15T). */
        public static final double kBevelPinionTeeth = 15.0;

        /**
         * Gear reduction from motor to wheel.
         */
        public static final double kReduction =
            (kBevelGearTeeth * kSpurTeeth) / (kPinionTeeth * kBevelPinionTeeth);

        /** Moment of inertia of drive system. */
        public static final double kMoI = 1.91e-4;

        /** Idle mode set to brake (prevents coasting when disabled). */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    /**
     * Turn motor (NEO 550) configuration.
     *
     * <p>Controls module orientation via position closed-loop with absolute encoder feedback.
     */
    public static final class TurnMotor {

        private TurnMotor() {}

        /** Moment of inertia of steering system. */
        public static final double kMoI = 2.17e-5;

        /** Gear reduction from turn motor to module rotation. */
        public static final double kReduction = 9424. / 203.;

        /** Idle mode set to brake (holds module angle when stopped). */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    /**
     * Drive encoder conversion factors.
     *
     * <p>Converts NEO internal encoder readings to wheel motion units.
     */
    public static final class DriveEncoder {

        private DriveEncoder() {}

        /**
         * Position conversion factor (motor rotations → wheel radians).
         */
        public static final double kPositionFactor =
            kTau / DriveMotor.kReduction;

        /**
         * Velocity conversion factor (motor RPM → wheel rad/s).
         */
        public static final double kVelocityFactor = kPositionFactor / 60.0;
    }

    /**
     * Turn encoder conversion factors.
     *
     * <p>Converts NEO 550 absolute encoder readings to module angle units.
     */
    public static final class TurnEncoder {

        private TurnEncoder() {}

        /**
         * Position conversion factor (motor rotations → motor radians).
         *
         * <p>Absolute encoder directly measures motor shaft, not geared output.
         */
        public static final double kPositionFactor = kTau;

        /**
         * Velocity conversion factor (motor RPM → motor rad/s).
         */
        public static final double kVelocityFactor = kPositionFactor / 60.0;

        /**
         * Invert the turn encoder.
         * <p><strong>Never change this. Ever.</strong> Changing this once lost two Neo 550s
         */
        public static final boolean kInverted = true;
    }

    /**
     * Wheel physical specifications.
     *
     * <p>Determines maximum achievable speeds and traction limits.
     */
    public static final class Wheel {

        private Wheel() {}

        /** Radius of the wheel. */
        public static final double kRadius = Inches.of(1.5).in(Meters);

        /**
         * Angular free speed of the wheel (radians/second).
         */
        public static final double kFreeSpeedAngular =
            MotorConstants.Neo.kFreeSpeed / DriveMotor.kReduction;

        /**
         * Linear free speed of the wheel.
         */
        public static final double kFreeSpeedLinear =
            kFreeSpeedAngular * kRadius;

        /** Estimated coefficient of friction for wheel on carpet. */
        public static final double kFrictionCoefficient = 1.3;
    }

    /**
     * Maximum robot speeds.
     *
     * <p>Defines velocity limits for trajectory planning and driver input scaling.
     */
    public static final class MaxSpeed {

        private MaxSpeed() {}

        /** Maximum linear speed. */
        public static final double kLinear = 4.804;

        /** Maximum angular speed. */
        public static final double kAngular = 12.440;
    }
}
