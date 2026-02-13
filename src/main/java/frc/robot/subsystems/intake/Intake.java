package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase 
{
    private static Intake instance;
    private TalonFX main;

    private DCMotorSim mainSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.Intake.GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));
    
   
    private Intake()
    {
        main = new TalonFX(Constants.Intake.ID);
        config();
        
        if (Robot.isSimulation())
        {
            main.getSimState().Orientation = Constants.Intake.MECHANICAL_ORIENTATION;
            main.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    private void config()
    {
        main.clearStickyFaults();
        TalonFXConfiguration mainConfig = new TalonFXConfiguration();

        mainConfig.Feedback.SensorToMechanismRatio = Constants.Intake.GEAR_RATIO;

        mainConfig.MotorOutput.Inverted = Constants.Intake.INVERTED;
        mainConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        mainConfig.Slot0.kP = Constants.Intake.KP;
        mainConfig.Slot0.kI = Constants.Intake.KI;
        mainConfig.Slot0.kD = Constants.Intake.KD;
        mainConfig.Slot0.kS = Constants.Intake.KS;
        mainConfig.Slot0.kV = Constants.Intake.KV;
        mainConfig.Slot0.kA = Constants.Intake.KA;

        mainConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        mainConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        mainConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.STATOR_CURRENT_LIMIT;
        mainConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.SUPPLY_CURRENT_LIMIT;

        main.getConfigurator().apply(mainConfig);
    }
   
    public Voltage getMainVoltage()
    {
        return main.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getMainVelocity()
    {
        return main.getVelocity().getValue();
    }
   
    public void setMainVoltage (Voltage voltage)
    {
        main.setControl(new VoltageOut(voltage));
    }

    public void setMainVelocity (AngularVelocity velocity)
    {
        //System.out.println("Velocity set to " + velocity);
        main.setControl(new VelocityVoltage(velocity));
    }

    public void setMainDutyCycle(double velocity) 
    {
        main.setControl(new DutyCycleOut(velocity));
    }
    
    @Override
    public void simulationPeriodic ()
    {
        TalonFXSimState simState = main.getSimState();

        // set the supply voltage of the TalonFX
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        mainSim.setInputVoltage(simState.getMotorVoltageMeasure().in(Volts));
        mainSim.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        simState.setRawRotorPosition(mainSim.getAngularPosition().times(Constants.Intake.GEAR_RATIO));
        simState.setRotorVelocity(mainSim.getAngularVelocity().times(Constants.Intake.GEAR_RATIO));
    }
    
    /*
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->motor.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Intake")
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
        */

    public static Intake getInstance()
    {
        if(instance == null) instance = new Intake();
        return instance;
    }
}
