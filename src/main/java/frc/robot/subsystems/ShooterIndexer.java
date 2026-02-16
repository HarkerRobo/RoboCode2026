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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.VoltageOut;

public class ShooterIndexer extends SubsystemBase
{
    
    private static ShooterIndexer instance;

    private static TalonFX motor;
    
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.ShooterIndexer.GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));

    private ShooterIndexer()
    {
        motor = new TalonFX(Constants.ShooterIndexer.ID);
        config();
        
        if (Robot.isSimulation())
        {
            motor.getSimState().Orientation = Constants.ShooterIndexer.MECHANICAL_ORIENTATION;
            motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    private void config() 
    {
        motor.clearStickyFaults();

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = Constants.ShooterIndexer.KG;
        slot0Configs.kS = Constants.ShooterIndexer.KS;
        slot0Configs.kV = Constants.ShooterIndexer.KV;
        slot0Configs.kA = Constants.ShooterIndexer.KA;
        slot0Configs.kP = Constants.ShooterIndexer.KP;
        slot0Configs.kI = Constants.ShooterIndexer.KI;
        slot0Configs.kD = Constants.ShooterIndexer.KD;

        talonFXConfigs.CurrentLimits.StatorCurrentLimit = Constants.ShooterIndexer.STATOR_CURRENT_LIMIT;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = Constants.ShooterIndexer.SUPPLY_CURRENT_LIMIT;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXConfigs.MotorOutput.Inverted = Constants.ShooterIndexer.INVERTED;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        talonFXConfigs.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        talonFXConfigs.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        talonFXConfigs.Feedback.SensorToMechanismRatio = Constants.ShooterIndexer.GEAR_RATIO;


        /* // add if necessary
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ShooterIndexer.MM_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.ShooterIndexer.MM_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.ShooterIndexer.MM_JERK;
        */

        motor.getConfigurator().apply(talonFXConfigs);
    }

    public void setVelocity(AngularVelocity velocity) 
    {
        motor.set(velocity.in(RotationsPerSecond));
    }

    public AngularVelocity getVelocity() 
    {
        return motor.getVelocity().getValue();
    }

    public void setVoltage(Voltage voltage) 
    {
        motor.setVoltage(voltage.in(Volts));
    }

    public Voltage getVoltage() 
    {
        return motor.getMotorVoltage().getValue();
    }

    public double getDutyCycle()
    {
        return motor.getDutyCycle().getValueAsDouble();
    }
    
    @Override
    public void simulationPeriodic ()
    {
        TalonFXSimState simState = motor.getSimState();

        // set the supply voltage of the TalonFX
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        sim.setInputVoltage(simState.getMotorVoltageMeasure().in(Volts));
        sim.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        simState.setRawRotorPosition(sim.getAngularPosition().times(Constants.ShooterIndexer.GEAR_RATIO));
        simState.setRotorVelocity(sim.getAngularVelocity().times(Constants.ShooterIndexer.GEAR_RATIO));
    }
    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->motor.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("ShooterIndexer")
                .voltage(getVoltage())
                .angularPosition(motor.getPosition().getValue())
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

    /**
     * Singleton code
     */
    public static ShooterIndexer getInstance() {
        if (instance == null) instance = new ShooterIndexer();
        return instance;
    }
}
