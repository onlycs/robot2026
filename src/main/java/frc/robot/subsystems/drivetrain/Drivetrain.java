package frc.robot.subsystems.drivetrain;

import static frc.robot.constants.SwerveConstants.*;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SwerveConstants.MaxSpeed;
import frc.robot.constants.SwerveConstants.ModuleId;
import frc.robot.constants.VisionConstants.VisionMeasurement;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.module.ModuleIO;
import frc.robot.subsystems.photon.CameraPhoton;
import frc.robot.subsystems.photon.CameraReplay;
import frc.robot.subsystems.photon.Photon;
import frc.robot.subsystems.quest.Quest;
import frc.robot.subsystems.quest.QuestIO;
import frc.robot.util.AdvantageUtil;
import frc.robot.util.AllianceUtil;
import frc.robot.util.IterUtil;
import frc.robot.util.RateLimiter;
import frc.robot.util.Vec2;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.Stream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * The swerve drivetrain subsystem.
 *
 * <p>This subsystem handles all robot movement through four independent swerve modules.
 * It manages:
 * <ul>
 *   <li>Module velocity and angle control
 *   <li>Gyroscope-based rotation tracking
 *   <li>Pose estimation using swerve kinematics and vision measurements
 *   <li>Multiple drive control modes (field-relative, robot-relative, alliance-relative)
 *   <li>Dashboard visualization via the Field2d widget and Elastic dashboard
 * </ul>
 *
 * <p>The drivetrain uses a dependency injection pattern to support both real hardware
 * (NavX gyro, Spark motor controllers) and replay/simulation alternatives.
 */
public class Drivetrain extends SubsystemBase {

    /** Front-left swerve module. */
    final ModuleIO frontLeft;

    /** Front-right swerve module. */
    final ModuleIO frontRight;

    /** Rear-left swerve module. */
    final ModuleIO rearLeft;

    /** Rear-right swerve module. */
    final ModuleIO rearRight;

    /** Gyroscope for absolute rotation tracking. */
    final GyroIO gyro;

    /** Pose estimator that combines encoder and gyro data with vision measurements. */
    final SwerveDrivePoseEstimator odometry;

    /** Field visualization widget for SmartDashboard and Elastic dashboard. */
    final Field2d field;

    /** Rate limiter for controlling acceleration on drive inputs. */
    final RateLimiter slew;

    /** The Meta Quest 3S, our primary vision tool */
    final Quest quest;

    /** List of measurements to calibrate the starting Pose from */
    final List<VisionMeasurement> calibrators = new ArrayList<>();

    /** PhotonVision, to calibrate the starting pose of the quest */
    final Photon photon = new Photon(
        this.calibrators::add,
        AdvantageUtil.match(CameraPhoton::new, CameraReplay::new)
    );

    /**
     * Constructs a new Drivetrain subsystem.
     *
     * Creates all four swerve modules, initializes the pose estimator, and sets up
     * the gyroscope. The actual hardware implementations are provided via dependency injection.
     *
     * @param moduleFactory A factory function that creates swerve modules based on their configuration
     * @param gyro The gyroscope implementation (NavX or GyroReplay)
     * @param quest The Quest IO implementation (Meta3S or QuestReplay)
     */
    public Drivetrain(
        Function<ModuleId, ModuleIO> moduleFactory,
        GyroIO gyro,
        QuestIO quest
    ) {
        this.gyro = gyro;
        this.quest = new Quest(quest, this::addVisionMeasurement);

        // Create all four swerve modules using the factory
        frontLeft = moduleFactory.apply(ModuleId.kFrontLeft);
        frontRight = moduleFactory.apply(ModuleId.kFrontRight);
        rearLeft = moduleFactory.apply(ModuleId.kRearLeft);
        rearRight = moduleFactory.apply(ModuleId.kRearRight);

        // Initialize the pose estimator with current module positions
        odometry = new SwerveDrivePoseEstimator(
            kKinematics,
            gyro.heading(),
            modulePositions(),
            new Pose2d()
        );

        // Create rate limiter for smooth acceleration control
        slew = new RateLimiter(
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kOrthogonal,
            ControlConstants.SlewRateLimit.kRotation
        );

        field = new Field2d();

        // Reset the gyro to zero
        this.gyro.reset(new Rotation2d());

        // Configure the Elastic dashboard swerve drive widget
        SmartDashboard.putData("SwerveDrive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            // Display each module's angle and velocity
            IterUtil.zipThen(
                Arrays.stream(modules()),
                Stream.of(
                    "Front Left",
                    "Front Right",
                    "Back Left",
                    "Back Right"
                ),
                (module, label) -> {
                    builder.addDoubleProperty(
                        label + " Angle",
                        () -> module.getState().angle.getRadians(),
                        null
                    );
                    builder.addDoubleProperty(
                        label + " Velocity",
                        () -> module.getState().speedMetersPerSecond,
                        null
                    );
                }
            );

            // Display robot heading (corrected for alliance if applicable)
            builder.addDoubleProperty(
                "Robot Angle",
                () -> {
                    Rotation2d ang = heading();
                    Rotation2d fixed = AllianceUtil.autoflip(ang).orElse(ang);
                    return fixed.getRadians();
                },
                null
            );
        });

        // Register with AdvantageKit
        AutoLogOutputManager.addObject(this);
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_RobotDrive,
            FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit
        );
    }

    /**
     * Gets all swerve modules on the robot.
     * @return An array of all modules in order: [frontLeft, frontRight, rearLeft, rearRight]
     */
    public ModuleIO[] modules() {
        return new ModuleIO[] { frontLeft, frontRight, rearLeft, rearRight };
    }

    /**
     * Gets the current position of each swerve module.
     *
     * Used by the odometry system to track robot movement.
     *
     * @return An array of module positions in the same order as modules()
     */
    @AutoLogOutput
    public SwerveModulePosition[] modulePositions() {
        return Arrays.stream(modules())
            .map(ModuleIO::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }

    /**
     * Gets the current velocity and angle of each swerve module.
     *
     * @return An array of module states in the same order as modules()
     */
    @AutoLogOutput
    public SwerveModuleState[] moduleStates() {
        return Arrays.stream(modules())
            .map(ModuleIO::getState)
            .toArray(SwerveModuleState[]::new);
    }

    /**
     * Gets the overall velocity of the robot's chassis.
     *
     * @return The current chassis speeds (vx, vy, omega)
     */
    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds() {
        return kKinematics.toChassisSpeeds(moduleStates());
    }

    /**
     * Commands robot-relative swerve drive with specified speeds.
     *
     * <p>See {@link #drive(ChassisSpeeds)} for field-relative driving.
     *
     * @param speeds The desired speeds for the robot to move at
     */
    public void set(ChassisSpeeds speeds) {
        // Convert to velocity vector for speed limiting
        Vec2 vel = new Vec2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        // Limit to maximum linear speed.
        // Can't limit components because, e.g., <100%X, 100%Y> = sqrt(2)*100% total speed.
        if (vel.mag() > MaxSpeed.kLinear) {
            vel = vel.norm().mul(MaxSpeed.kLinear);
            speeds.vxMetersPerSecond = vel.x;
            speeds.vyMetersPerSecond = vel.y;
        }

        // Limit to maximum angular speed
        speeds.omegaRadiansPerSecond = MathUtil.clamp(
            speeds.omegaRadiansPerSecond,
            -MaxSpeed.kAngular,
            MaxSpeed.kAngular
        );

        // Discretize speeds for more accurate control (matches 20ms robot tick)
        ChassisSpeeds discrete = ChassisSpeeds.discretize(speeds, 0.02);

        // Convert chassis speeds to individual module states
        SwerveModuleState[] states = kKinematics.toSwerveModuleStates(discrete);

        // Ensure all wheels are commanded within maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MaxSpeed.kLinear);

        // Log for debugging and analysis
        Logger.recordOutput("Drivetrain/StateSetpoints", states);
        Logger.recordOutput("Drivetrain/SpeedSetpoint", discrete);

        // Command each module to its desired state
        IterUtil.zipThen(
            Arrays.stream(modules()),
            Arrays.stream(states),
            ModuleIO::setDesiredState
        );
    }

    /**
     * Gets the estimated position of the robot on the field.
     *
     * This is updated using encoder/gyro data and enhanced with vision measurements.
     *
     * @return The robot's current pose (x, y, rotation)
     */
    @AutoLogOutput
    public Pose2d pose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Gets the robot's current heading (rotation).
     *
     * Equivalent to {@link #pose()}.getRotation() but slightly more convenient.
     *
     * @return The robot's rotation angle
     */
    public Rotation2d heading() {
        return pose().getRotation();
    }

    /**
     * Commands swerve drive, field-relative with specified speeds.
     *
     * <p>Use {@link #set(ChassisSpeeds)} for robot-relative driving.
     *
     * @param speeds The desired chassis speeds
     */
    public void drive(ChassisSpeeds speeds) {
        set(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading()));
    }

    /**
     * Commands swerve drive with power inputs (normalized to [-1, 1]).
     *
     * <p>Supports field-relative driving. See {@link #set(ChassisSpeeds)} for robot-relative driving.
     *
     * @param pow The X and Y power inputs ([-1, 1])
     * @param rotPower The rotational power input ([-1, 1])
     */
    public void drive(Vec2 pow, double rotPower) {
        // Scale to maximum velocities
        Vec2 vel = pow.mul(MaxSpeed.kLinear);
        double rot = rotPower * MaxSpeed.kAngular;

        // NOTE: No need to speed limit here; set() will handle that.
        drive(new ChassisSpeeds(vel.x, vel.y, rot));
    }

    /**
     * Commands swerve drive from an Xbox controller.
     *
     * @param controller The Xbox controller to read input from
     */
    public void drive(CommandXboxController controller) {
        Vec2 linear = new Vec2(controller.getLeftX(), controller.getLeftY());

        // Apply deadband to joystick inputs
        double xymag = MathUtil.applyDeadband(
            linear.mag(),
            IOConstants.Controller.kDeadband
        );
        double rot = MathUtil.applyDeadband(
            controller.getRightX(),
            IOConstants.Controller.kDeadband
        );

        Vec2 input = linear.norm().mul(xymag);

        // Apply squared inputs for finer low-speed control
        Vec2 input2 = input.mul(input.abs());
        double rot2 = rot * Math.abs(rot);

        // Apply rate limiting for smooth acceleration
        RateLimiter.Outputs limited = slew.calculate(input2, rot2);

        /*
         * Controller to WPI coordinate system conversion:
         *
         * Controller coordinates:    WPI coordinates:
         *   +X = right                 +X = forward
         *   +Y = down                  +Y = left
         *   +Rot = clockwise           +Rot = counter-clockwise
         *
         * Transformation needed:
         *   +Yc -> -Xw  (down stick -> forward motion negation)
         *   +Xc -> -Yw  (right stick -> left motion negation)
         *   +Rotc -> -Rotw (negate rotation for ccw-positive)
         */
        Vec2 linearcmd = new Vec2(-limited.vel().y, -limited.vel().x);
        double rotcmd = -limited.rot();

        /*
         * Field-centric drive coordinate transformation based on alliance color.
         *
         * BLUE ALLIANCE (origin on driver station side):
         *   - Joystick forward (+Y) → Robot moves toward opposing alliance (+X field coords)
         *   - No transformation needed - joystick and field coordinates align naturally
         *
         * RED ALLIANCE (origin on opposing alliance side):
         *   - Joystick forward (+Y) → Robot should move toward opposing alliance (-X field coords)
         *   - Linear commands must be inverted to maintain driver perspective
         *   - This ensures drivers have consistent controls regardless of alliance color
         */
        Optional<Boolean> invert = AllianceUtil.invert();
        if (invert.orElse(false)) linearcmd = linearcmd.neg(); // Invert for red alliance, skip if no FMS/DS

        drive(linearcmd, rotcmd);
    }

    /**
     * Incorporates a vision measurement into the pose estimate.
     *
     * Allows the pose estimator to correct odometry drift using camera-based pose detection.
     * The measurement includes position, rotation, timestamp, and measurement covariance.
     *
     * @param measurement The vision measurement to incorporate
     */
    public void addVisionMeasurement(VisionMeasurement measurement) {
        odometry.addVisionMeasurement(
            measurement.estimate2(),
            measurement.timestamp(),
            measurement.stdDevs()
        );
    }

    /**
     * Gets the field visualization widget.
     *
     * This can be used to display the robot position and trajectory on SmartDashboard
     * or Elastic dashboard.
     *
     * @return The Field2d widget
     */
    public Field2d getField() {
        return field;
    }

    /**
     * Calibrate the initial pose of the robot
     *
     * Photon/AprilTag measurements are recorded and collected while the robot is disabled.
     * Once enabled, we average the latest few measurements and set our starting pose to that.
     */
    private void calibrate() {
        if (calibrators.isEmpty()) return;

        // Average all calibrator measurements
        double x = 0;
        double y = 0;
        double sin = 0;
        double cos = 0;
        int n = 0;

        for (VisionMeasurement vm : calibrators) {
            double age = Timer.getFPGATimestamp() - vm.timestamp();
            if (age > 15) continue; // Ignore old measurements

            Pose2d est = vm.estimate2();

            x += est.getX();
            y += est.getY();
            sin += est.getRotation().getSin();
            cos += est.getRotation().getCos();
            n++;
        }

        x /= n;
        y /= n;
        sin /= n;
        cos /= n;

        Pose2d avg = new Pose2d(x, y, new Rotation2d(cos, sin));

        // Reset odometry and gyro to the averaged pose
        reset(avg);

        // Clear calibrators for next use
        calibrators.clear();
    }

    /**
     * Resets all of the drivetrain's sensors to the specified pose.
     *
     * @param pose The new pose to set
     */
    public void reset(Pose2d pose) {
        gyro.reset(pose.getRotation());
        quest.reset(pose);
        odometry.resetPosition(pose.getRotation(), modulePositions(), pose);
    }

    /**
     * Resets all of the drivetrain's sensors to the specified rotation.
     *
     * @param rotation The new rotation to set
     */
    public void reset(Rotation2d rotation) {
        Pose2d current = pose();
        Pose2d next = new Pose2d(current.getTranslation(), rotation);
        reset(next);
    }

    /**
     * Periodic update method called every 20ms.
     *
     * <p>Updates all drivetrain components and logs data for analysis:
     * <ul>
     *   <li>Reads gyroscope heading
     *   <li>Updates all swerve modules
     *   <li>Updates pose estimator
     *   <li>Logs data to AdvantageKit
     *   <li>Updates dashboard displays
     * </ul>
     */
    @Override
    public void periodic() {
        // Collect updates when disabled; calibrate once enabled.
        // although calibrate() is called many times, it only acts once
        // since we clear the calibrators list at the end of the method.
        if (DriverStation.isDisabled()) photon.update();
        else calibrate();

        // Update other subsystems
        quest.update();
        gyro.update();
        Arrays.stream(modules()).forEach(ModuleIO::update);

        // Record gyro heading
        Logger.processInputs("Drivetrain/Gyro", gyro.data);
        Arrays.stream(modules()).forEach(module -> {
            Logger.processInputs(
                "Drivetrain/Module/%d".formatted(module.id.value),
                module.data
            );
        });

        Rotation2d heading;
        Rotation2d est = quest.heading().orElse(null);

        // Prefer gyro if connected, else fall back to Quest heading
        // If neither is available, use zero
        if (gyro.data.connected) heading = gyro.heading();
        else if (est != null) heading = est;
        else heading = new Rotation2d();

        // Update odometry with latest measurements
        try {
            odometry.update(heading, modulePositions());
        } catch (Exception ignored) {}

        // Log module states and chassis speeds
        Logger.recordOutput("Drivetrain/ModuleStates", moduleStates());
        Logger.recordOutput("Drivetrain/ChassisSpeeds", getChassisSpeeds());

        // Log active command for debugging
        AdvantageUtil.logActiveCommand(this);

        // Update dashboard field display
        field.setRobotPose(pose());
        SmartDashboard.putData("Field", field);
    }
}
