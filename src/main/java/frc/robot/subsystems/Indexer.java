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
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

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
        main = new TalonFX(Constants.Indexer.MAIN_ID, Constants.CAN_CHAIN);
        side = new TalonFX(Constants.Indexer.SIDE_ID, Constants.CAN_CHAIN);
        config();
        
        if (isSimulated())
        {
            main.getSimState().Orientation = Constants.Indexer.MAIN_MECHANICAL_ORIENTATION;
            main.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            
            side.getSimState().Orientation = Constants.Indexer.SIDE_MECHANICAL_ORIENTATION;
            side.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    private void config() {
        main.clearStickyFaults();

        var mainConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var mainSlot0Configs = mainConfigs.Slot0;
        mainSlot0Configs.kS = Constants.Indexer.MAIN_KS;
        mainSlot0Configs.kV = Constants.Indexer.MAIN_KV;
        mainSlot0Configs.kA = Constants.Indexer.MAIN_KA;
        mainSlot0Configs.kP = Constants.Indexer.MAIN_KP;
        mainSlot0Configs.kI = Constants.Indexer.MAIN_KI;
        mainSlot0Configs.kD = Constants.Indexer.MAIN_KD;

        mainConfigs.CurrentLimits.StatorCurrentLimit = Constants.Indexer.MAIN_STATOR_CURRENT_LIMIT;
        mainConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        mainConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.MAIN_SUPPLY_CURRENT_LIMIT;
        mainConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        mainConfigs.MotorOutput.Inverted = Constants.Indexer.MAIN_INVERTED;
        mainConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        mainConfigs.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        mainConfigs.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        mainConfigs.Feedback.SensorToMechanismRatio = Constants.Indexer.MAIN_GEAR_RATIO;


        /* // add if necessary
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Indexer.MM_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Indexer.MM_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.Indexer.MM_JERK;
        */

        main.getConfigurator().apply(mainConfigs);
        
        side.clearStickyFaults();

        var sideConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var sideSlot0Configs = sideConfigs.Slot0;
        sideSlot0Configs.kS = Constants.Indexer.SIDE_KS;
        sideSlot0Configs.kV = Constants.Indexer.SIDE_KV;
        sideSlot0Configs.kA = Constants.Indexer.SIDE_KA;
        sideSlot0Configs.kP = Constants.Indexer.SIDE_KP;
        sideSlot0Configs.kI = Constants.Indexer.SIDE_KI;
        sideSlot0Configs.kD = Constants.Indexer.SIDE_KD;

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

    public double getMainDutyCycle()
    {
        return main.getDutyCycle().getValueAsDouble();
    }
    
    public AngularVelocity getSideVelocity() 
    {
        return side.getVelocity().getValue();
    }
    
    public Voltage getSideVoltage() 
    {
        return side.getMotorVoltage().getValue();
    }

    public double getSideDutyCycle()
    {
        return side.getDutyCycle().getValueAsDouble();
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

    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INDEXER_INDEX) == SubsystemStatus.Simulated;
    }
    
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INDEXER_INDEX) == SubsystemStatus.Disabled;
    }
    
    /*
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->main.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Indexer")
                .voltage(getVoltage())
                .angularPosition(main.getPosition().getValue())
                .angularVelocity(getVelocity()),
        this)
    );

    public Command sysIdQuasistatic (SysIdRoutine.Direction direction)
    {
        return sysId.quasistatic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    
    public Command sysIdDynamic (SysIdRoutine.Direction direction)
    {
        return sysId.dynamic(direction).withName("SysId Q" + (direction == SysIdRoutine.Direction.kForward ? "F" : "R"));
    }
    */

    /**
     * Singleton code
     */
    public static Indexer getInstance() {
        if (instance == null) instance = new Indexer();
        return instance;
    }
}
