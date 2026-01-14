package frc.robot.subsystems.kitbot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;

/**
 * KitBot IO implementation using Spark Max motor controllers.
 *
 * <p>This class interfaces with Spark Max motor controllers to control
 * the feeder and intake motors of the KitBot superstructure.
 */
public class KitBotSpark extends KitBotIO {

    /** Spark Max motor controller for the feeder mechanism */
    final SparkMax feeder;

    /** Spark Max motor controller for the intake mechanism */
    final SparkMax intake;

    /**
     * Constructs a KitBotSpark IO implementation.
     *
     * <p>Initializes the Spark Max motor controllers for the feeder and intake,
     * and configures them with predefined settings.
     */
    public KitBotSpark() {
        this.feeder = new SparkMax(
            IOConstants.KitBot.kFeeder,
            MotorType.kBrushed
        );
        this.intake = new SparkMax(
            IOConstants.KitBot.kIntake,
            MotorType.kBrushed
        );

        feeder.configure(
            SparkConfigConstants.KitBot.kFeeder,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );
        intake.configure(
            SparkConfigConstants.KitBot.kIntake,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );
    }

    @Override
    public void setFeeder(double power) {
        feeder.set(power);
    }

    @Override
    public void setIntake(double power) {
        intake.set(power);
    }

    @Override
    public void update() {
        // Feeder readings
        data.feederConnected = feeder.getLastError() != REVLibError.kOk;
        data.feederVoltage = Volts.of(feeder.getBusVoltage());
        data.feederCurrent = Amps.of(feeder.getOutputCurrent());

        // Intake readings
        data.intakeConnected = intake.getLastError() != REVLibError.kOk;
        data.intakeVoltage = Volts.of(intake.getBusVoltage());
        data.intakeCurrent = Amps.of(intake.getOutputCurrent());
    }
}
