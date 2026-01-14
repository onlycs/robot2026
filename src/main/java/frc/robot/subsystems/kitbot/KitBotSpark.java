package frc.robot.subsystems.kitbot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;

public class KitBotSpark extends KitBotIO {

    final SparkMax feeder;
    final SparkMax intake;

    final RelativeEncoder feederEncoder;
    final RelativeEncoder intakeEncoder;

    public KitBotSpark() {
        this.feeder = new SparkMax(
            IOConstants.KitBot.kFeeder,
            MotorType.kBrushed
        );
        this.intake = new SparkMax(
            IOConstants.KitBot.kIntake,
            MotorType.kBrushed
        );

        this.feederEncoder = feeder.getEncoder();
        this.intakeEncoder = intake.getEncoder();

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

    /** Set feeder motor power, [-1, 1] */
    @Override
    public void setFeeder(double power) {
        feeder.set(power);
    }

    /** Set intake motor power, [-1, 1] */
    @Override
    public void setIntake(double power) {
        intake.set(power);
    }

    /**
     * Updates KitBot sensor readings.
     */
    @Override
    public void update() {
        // Feeder readings
        data.feederConnected = feeder.getLastError() != REVLibError.kOk;
        data.feederPosition = Radians.of(feederEncoder.getPosition());
        data.feederVelocity = RadiansPerSecond.of(feederEncoder.getVelocity());
        data.feederVoltage = Volts.of(feeder.getBusVoltage());
        data.feederCurrent = Amps.of(feeder.getOutputCurrent());

        // Intake readings
        data.intakeConnected = intake.getLastError() != REVLibError.kOk;
        data.intakePosition = Radians.of(intakeEncoder.getPosition());
        data.intakeVelocity = RadiansPerSecond.of(intakeEncoder.getVelocity());
        data.intakeVoltage = Volts.of(intake.getBusVoltage());
        data.intakeCurrent = Amps.of(intake.getOutputCurrent());
    }
}
