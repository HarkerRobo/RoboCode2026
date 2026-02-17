package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;

public class Intake extends SubsystemBase 
{
    private static Intake instance;
    private TalonFX motor;

    private DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.Intake.GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));

   
    private Intake()
    {
        motor = new TalonFX(Constants.Intake.ID);
        config();
        
        if (isSimulated())
        {
            motor.getSimState().Orientation = Constants.Intake.MECHANICAL_ORIENTATION;
            motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    private void config()
    {
        motor.clearStickyFaults();
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.Feedback.SensorToMechanismRatio = Constants.Intake.GEAR_RATIO;

        motorConfig.MotorOutput.Inverted = Constants.Intake.INVERTED;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motorConfig.Slot0.kP = Constants.Intake.KP;
        motorConfig.Slot0.kI = Constants.Intake.KI;
        motorConfig.Slot0.kD = Constants.Intake.KD;
        motorConfig.Slot0.kS = Constants.Intake.KS;
        motorConfig.Slot0.kV = Constants.Intake.KV;
        motorConfig.Slot0.kA = Constants.Intake.KA;

        motorConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        motorConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        motorConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(motorConfig);
    }
   
    public Voltage getVoltage()
    {
        return motor.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getVelocity()
    {
        return motor.getVelocity().getValue();
    }
   
    public void setVoltage (Voltage voltage)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Intake");
            return;
        }
        motor.setControl(new VoltageOut(voltage));
    }

    public void setVelocity (AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Intake");
            return;
        }
        motor.setControl(new VelocityVoltage(velocity));
    }

    public void setDutyCycle(double velocity) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Intake");
            return;
        }
        motor.setControl(new DutyCycleOut(velocity));
    }
    
    @Override
    public void periodic ()
    {
        if (isSimulated())
        {
            TalonFXSimState simState = motor.getSimState();

            // set the supply voltage of the TalonFX
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            motorSim.setInputVoltage(simState.getMotorVoltageMeasure().in(Volts));
            motorSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            simState.setRawRotorPosition(motorSim.getAngularPosition().times(Constants.Intake.GEAR_RATIO));
            simState.setRotorVelocity(motorSim.getAngularVelocity().times(Constants.Intake.GEAR_RATIO));
        }
    }
   
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INTAKE_INDEX) == SubsystemStatus.Simulated;
    }
    
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INTAKE_INDEX) == SubsystemStatus.Disabled;
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
