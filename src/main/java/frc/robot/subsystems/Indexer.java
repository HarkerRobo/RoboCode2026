package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class Indexer extends SubsystemBase{
    
    private static Indexer instance;

    private static TalonFX motor;

    private Indexer() { 
        motor = new TalonFX(Constants.Indexer.MOTOR_ID);
        config();
    }

    private void config() {
        motor.clearStickyFaults();

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = Constants.Indexer.kG;
        slot0Configs.kS = Constants.Indexer.kS;
        slot0Configs.kV = Constants.Indexer.kV;
        slot0Configs.kA = Constants.Indexer.kA;
        slot0Configs.kP = Constants.Indexer.kP;
        slot0Configs.kI = Constants.Indexer.kI;
        slot0Configs.kD = Constants.Indexer.kD;

        talonFXConfigs.CurrentLimits.StatorCurrentLimit = Constants.Indexer.STATOR_CURRENT_LIMIT;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.SUPPLY_CURRENT_LIMIT;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXConfigs.MotorOutput.Inverted = Constants.Indexer.INVERTED;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        talonFXConfigs.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        talonFXConfigs.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        talonFXConfigs.Feedback.SensorToMechanismRatio = Constants.Indexer.GEAR_RATIO;


        /* // add if necessary
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Indexer.MM_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Indexer.MM_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.Indexer.MM_JERK;
        */

        motor.getConfigurator().apply(talonFXConfigs);
    }

    public void setVelocity(AngularVelocity velocity) {
        motor.set(velocity.in(RotationsPerSecond));
    }

    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    public double dutyCycle()
    {
        return motor.getDutyCycle().getValueAsDouble();
    }

    /**
     * Singleton code
     */
    public static Indexer getInstance() {
        if (instance == null) instance = new Indexer();
        return instance;
    }
}
