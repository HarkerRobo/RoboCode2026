package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;
import frc.robot.commands.intake.StartDefaultIntake;

public class Intake extends SubsystemBase 
{
    private static Intake instance;
    private TalonFX left;
    private TalonFX right;
    private double targetVelocity;

    private DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, Constants.Intake.LEFT_GEAR_RATIO),
        DCMotor.getKrakenX60Foc(1));

   
    /**
     * Initializes the intake motor and applies all configuration settings.
     * If running in simulation, sets up the TalonFX sim state so the virtual motor behaves correctly.
     */
    private Intake()
    {
        left = new TalonFX(Constants.Intake.LEFT_ID, Constants.CAN_SUPERSTRUCTURE);
        right = new TalonFX(Constants.Intake.RIGHT_ID, Constants.CAN_SUPERSTRUCTURE);
        config();
        
        if (isSimulated())
        {
            left.getSimState().Orientation = Constants.Intake.MECHANICAL_ORIENTATION;
            left.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    /**
     * Sets up the TalonFX with all the intake’s settings:
     * inversion, PID values, current limits, voltage limits, and sensor ratio.
     */
    private void config()
    {
        left.clearStickyFaults();
        right.clearStickyFaults();
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.Feedback.SensorToMechanismRatio = Constants.Intake.LEFT_GEAR_RATIO;
        leftConfig.MotorOutput.Inverted = Constants.Intake.LEFT_INVERTED;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        left.getConfigurator().apply(leftConfig);
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.Feedback.SensorToMechanismRatio = Constants.Intake.RIGHT_GEAR_RATIO;
        rightConfig.MotorOutput.Inverted = Constants.Intake.RIGHT_INVERTED;
        right.getConfigurator().apply(rightConfig);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motorConfig.Slot0.kP = Constants.Intake.KP;
        motorConfig.Slot0.kI = Constants.Intake.KI;
        motorConfig.Slot0.kD = Constants.Intake.KD;

        motorConfig.Slot0.kV = Constants.Intake.KV;

        motorConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        motorConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        motorConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        //left.getConfigurator().apply(motorConfig);
        right.getConfigurator().apply(motorConfig);

        left.setControl(new Follower(Constants.Intake.RIGHT_ID, MotorAlignmentValue.Opposed));
    }
    /**
    * Returns the target angular velocity.
    * Used if you need the velocity.
    */
    public AngularVelocity getTargetVelocity()
    {
        return RotationsPerSecond.of(targetVelocity);
    }
   
    /**
     * Returns the right motor voltage.
     */
    public Voltage getRightVoltage()
    {
        return right.getMotorVoltage().getValue();
    }
    
    /**
     * Returns the right motor velocity.
     * Represents the measured angular speed of the intake.
     */
    public AngularVelocity getRightVelocity()
    {
        return right.getVelocity().getValue();
    }

    /**
     * Returns the right stator current.
     * Indicates the electrical load on the motor.
     */
    public Current getRightStatorCurrent()
    {
        return right.getStatorCurrent().getValue();
    }
    
    /**
     * Returns the left motor voltage.
     * Useful for telemetry and debugging.
     */
    public Voltage getLeftVoltage()
    {
        return left.getMotorVoltage().getValue();
    }
    
    /**
     * Returns the left motor velocity.
     * Represents the measured angular speed of the intake.
     */
    public AngularVelocity getLeftVelocity()
    {
        return left.getVelocity().getValue();
    }

    /**
     * Returns the left stator current.
     * Indicates the electrical load on the motor.
     */
    public Current getLeftStatorCurrent()
    {
        return left.getStatorCurrent().getValue();
    }
    
    public boolean isStalling()
    {
        return Math.abs(left.getStatorCurrent().getValueAsDouble()) >= Constants.Intake.STALLING_CURRENT ||
            Math.abs(right.getStatorCurrent().getValueAsDouble()) >= Constants.Intake.STALLING_CURRENT;
    }
    
    /**
    * Pushes a specific voltage into the motor.
    * Blocked if disabled
    */
    public void setVoltage (Voltage voltage)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Intake");
            return;
        }
        right.setControl(new VoltageOut(voltage));
    }

    /**
    * Commands to hold a target velocity.
    * Automatically does feedforward and PID.
    */
    public void setVelocity (AngularVelocity velocity)
    {
        targetVelocity = velocity.in(RotationsPerSecond);
        if (isDisabled())
        {
            System.out.println("Quashing input to Intake");
            return;
        }
        right.setControl(new VelocityVoltage(velocity));
    }
    
    /**
     * When running in simulation, updates the DCMotorSim.
     * Pushes the new rotor position and velocity into the TalonFX sim state.
     */
    @Override
    public void periodic ()
    {
        // if (isStalling())
        // {
        //     CommandScheduler.getInstance().schedule(new StartDefaultIntake());
        // }

        if (isSimulated())
        {
            TalonFXSimState simState = right.getSimState();

            // set the supply voltage of the TalonFX
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            motorSim.setInputVoltage(simState.getMotorVoltageMeasure().in(Volts));
            motorSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            simState.setRawRotorPosition(motorSim.getAngularPosition().times(Constants.Intake.RIGHT_GEAR_RATIO));
            simState.setRotorVelocity(motorSim.getAngularVelocity().times(Constants.Intake.RIGHT_GEAR_RATIO));
        }
    }
   
    /**
     * Returns true if the subsystem is running in simulation.
     * Uses RobotContainer status to determine the mode.
     */
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INTAKE_INDEX) == SubsystemStatus.Simulated;
    }
    

    /**
     * Returns true if the subsystem is disabled.
     * Prevents motor commands from being applied.
     */
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.INTAKE_INDEX) == SubsystemStatus.Disabled;
    }


    /**
     * Returns the singleton Intake instance.
     * Creates the subsystem on first access.
     */
    public static Intake getInstance()
    {
        if(instance == null) instance = new Intake();
        return instance;
    }
}
