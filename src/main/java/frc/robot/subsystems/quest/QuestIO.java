package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.AutoLog;

/**
 * Abstract IO class for the QuestNav subsystem.
 *
 * <p>This class defines the interface for interacting with the QuestNav device,
 * including methods for updating sensor data and resetting the device's pose.
 *
 * <p>Concrete implementations of this class should handle the specifics of
 * communicating with the QuestNav hardware or replay implementations.
 */
public abstract class QuestIO {

    /** Auto-logged data structure for QuestNav data */
    @AutoLog
    public static class QuestData {

        QuestData() {}

        /** Connection status of the QuestNav device */
        public boolean connected = false;
        /** All of the pose frames read from the QuestNav device */
        public PoseFrame[] readings = new PoseFrame[0];
    }

    /** The current QuestNav data, since the last update() call */
    public final QuestDataAutoLogged data = new QuestDataAutoLogged();

    /** Updates this.data with the current QuestNav data. */
    public abstract void update();

    /**
     * Reset the <b>quest</b> to the specified pose.
     *
     * @param pose The reset pose of the <b>quest</b>
     */
    protected abstract void reset(Pose3d pose);
}
