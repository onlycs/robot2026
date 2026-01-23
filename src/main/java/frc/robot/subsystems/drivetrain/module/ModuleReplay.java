package frc.robot.subsystems.drivetrain.module;

import frc.robot.constants.SwerveConstants;

/**
 * Replay implementation of {@link ModuleIO} for log playback.
 *
 * <p>This class is used when replaying recorded robot data from AdvantageKit logs.
 * It does not interact with any real hardware or simulation - all module data
 * is read directly from logged {@link ModuleData}.
 *
 * <p>All methods are no-ops because all replayed data is immutable.
 */
public class ModuleReplay extends ModuleIO {

    public ModuleReplay(SwerveConstants.ModuleId id) {
        super(id);
    }

    @Override
    public void update() {}

    @Override
    public void setStateSetpoint(double driveVelocity, double turnPosition) {}
}
