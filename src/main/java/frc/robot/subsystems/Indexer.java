package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.VelocityVoltage;

public class Indexer extends SubsystemBase
{
    private static Indexer instance;

    private static TalonFX main;
    private static TalonFX side;
    
    private DCMotorSim mainSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.Indexer.MAIN_GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));
    
    private DCMotorSim sideSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.Indexer.SIDE_GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));

    private Indexer() { 
        main = new TalonFX(Constants.Indexer.MAIN_ID, Constants.CAN_SUPERSTRUCTURE);
        side = new TalonFX(Constants.Indexer.SIDE_ID, Constants.CAN_SUPERSTRUCTURE);
        config();
        
        if (isSimulated())
        {
            main.getSimState().Orientation = Constants.Indexer.MAIN_MECHANICAL_ORIENTATION;
            main.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            
            side.getSimState().Orientation = Constants.Indexer.SIDE_MECHANICAL_ORIENTATION;
            side.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    /**
     * Applies all TalonFX settings: PID values, current limits, inversion, voltage limits, and sensor ratio.
     * Ensures the indexer motor starts with the correct behavior before anything commands it.
     */
    private void config() {
        main.clearStickyFaults();

        TalonFXConfiguration mainConfigs = new TalonFXConfiguration();

        mainConfigs.Slot0.kP = Constants.Indexer.MAIN_KP;
        mainConfigs.Slot0.kI = Constants.Indexer.MAIN_KI;
        mainConfigs.Slot0.kD = Constants.Indexer.MAIN_KD;

        mainConfigs.CurrentLimits.StatorCurrentLimit = Constants.Indexer.MAIN_STATOR_CURRENT_LIMIT;
        mainConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        mainConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.MAIN_SUPPLY_CURRENT_LIMIT;
        mainConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        mainConfigs.MotorOutput.Inverted = Constants.Indexer.MAIN_INVERTED;
        mainConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        mainConfigs.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        mainConfigs.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        mainConfigs.Feedback.SensorToMechanismRatio = Constants.Indexer.MAIN_GEAR_RATIO;

        main.getConfigurator().apply(mainConfigs);
        
        side.clearStickyFaults();

        TalonFXConfiguration sideConfigs = new TalonFXConfiguration();

        // currently not in use
        sideConfigs.Slot0.kP = Constants.Indexer.SIDE_KP;
        sideConfigs.Slot0.kI = Constants.Indexer.SIDE_KI;
        sideConfigs.Slot0.kD = Constants.Indexer.SIDE_KD;

        sideConfigs.CurrentLimits.StatorCurrentLimit = Constants.Indexer.SIDE_STATOR_CURRENT_LIMIT;
        sideConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        sideConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.SIDE_SUPPLY_CURRENT_LIMIT;
        sideConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        sideConfigs.MotorOutput.Inverted = Constants.Indexer.SIDE_INVERTED;
        sideConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        sideConfigs.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        sideConfigs.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        sideConfigs.Feedback.SensorToMechanismRatio = Constants.Indexer.SIDE_GEAR_RATIO;

        side.getConfigurator().apply(sideConfigs);
    }

    // currently not in use
    public void setMainVelocity(AngularVelocity velocity) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Indexer");
            return;
        }
        main.setControl(new VelocityVoltage(velocity));
    }

    public void setMainVoltage(Voltage voltage) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Indexer");
            return;
        }
        main.setVoltage(voltage.in(Volts));
    }
    
    // currently not in use
    public void setSideVelocity(AngularVelocity velocity) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Indexer");
            return;
        }
        side.setControl(new VelocityVoltage(velocity));
    }

    public void setSideVoltage(Voltage voltage) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Indexer");
            return;
        }
        side.setVoltage(voltage.in(Volts));
    }
    
    public AngularVelocity getMainVelocity() 
    {
        return main.getVelocity().getValue();
    }

    public Voltage getMainVoltage() 
    {
        return main.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getSideVelocity() 
    {
        return side.getVelocity().getValue();
    }
    
    public Voltage getSideVoltage() 
    {
        return side.getMotorVoltage().getValue();
    }

    @Override
    public void periodic ()
    {
        if (isSimulated())
        {
            TalonFXSimState mainSimState = main.getSimState();

            // set the supply voltage of the TalonFX
            mainSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            mainSim.setInputVoltage(mainSimState.getMotorVoltageMeasure().in(Volts));
            mainSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            mainSimState.setRawRotorPosition(mainSim.getAngularPosition().times(Constants.Indexer.MAIN_GEAR_RATIO));
            mainSimState.setRotorVelocity(mainSim.getAngularVelocity().times(Constants.Indexer.MAIN_GEAR_RATIO));
            
            TalonFXSimState sideSimState = main.getSimState();

            // set the supply voltage of the TalonFX
            sideSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            sideSim.setInputVoltage(sideSimState.getMotorVoltageMeasure().in(Volts));
            sideSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            sideSimState.setRawRotorPosition(mainSim.getAngularPosition().times(Constants.Indexer.SIDE_GEAR_RATIO));
            sideSimState.setRotorVelocity(mainSim.getAngularVelocity().times(Constants.Indexer.SIDE_GEAR_RATIO));
        }
    }

    /**
     * Checks RobotContainer to see if this subsystem is running in simulation mode.
     * Controls whether the sim model runs.
     */
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INDEXER_INDEX) == SubsystemStatus.Simulated;
    }
    
    /**
     * Checks if the subsystem is marked disabled.
     * Blocks all motor commands when it is.
     */
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INDEXER_INDEX) == SubsystemStatus.Disabled;
    }
    
    public static Indexer getInstance() {
        if (instance == null) instance = new Indexer();
        return instance;
    }
}
