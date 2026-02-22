package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;

/**
 * controls the balls from intake to indexer
 */
public class Hopper extends SubsystemBase
{
    private static Hopper instance;
    private TalonFX master;
    /*
    private double desiredPosition; // rotations
    */

    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Hopper.GEAR_RATIO),
        DCMotor.getKrakenX60(1), 
        Constants.Hopper.MIN_POSITION, 
        Constants.Hopper.MAX_POSITION, 
        false, 
        Constants.Hopper.MIN_POSITION);

    private Hopper()
    {
        master = new TalonFX(Constants.Hopper.ID);
        
        if (isSimulated())
        {
            TalonFXSimState simState = master.getSimState();
            simState.Orientation = Constants.Hopper.MECHANICAL_ORIENTATION;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }

        config();
    }

    private void config()
    {
        master.clearStickyFaults();
        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.Feedback.SensorToMechanismRatio = Constants.Hopper.GEAR_RATIO;
        
        masterConfig.CurrentLimits.StatorCurrentLimit = Constants.Hopper.STATOR_CURRENT_LIMIT;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        masterConfig.CurrentLimits.SupplyCurrentLimit = Constants.Hopper.SUPPLY_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;


        /*
        masterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Hopper.MM_CRUISE_VELOCITY;
        masterConfig.MotionMagic.MotionMagicAcceleration = Constants.Hopper.MM_ACCELERATION;
        masterConfig.MotionMagic.MotionMagicJerk = Constants.Hopper.MM_JERK;
        */
        masterConfig.MotorOutput.Inverted = Constants.Hopper.INVERTED;

        masterConfig.Slot0.kP = Constants.Hopper.KP;
        masterConfig.Slot0.kI = Constants.Hopper.KI;
        masterConfig.Slot0.kD = Constants.Hopper.KD;

        masterConfig.Slot0.kG = Constants.Hopper.KS;
        masterConfig.Slot0.kV = Constants.Hopper.KV;
        masterConfig.Slot0.kV = Constants.Hopper.KA;

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        masterConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        masterConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        master.getConfigurator().apply(masterConfig);
    }


    /**
     * @return rotations
     */
    public Angle getPosition() 
    {
        return master.getPosition().getValue();
    }
    
    /**
     * @return rotations per second
     */
    public AngularVelocity getVelocity()
    {
        return master.getVelocity().getValue();
    }

    /**
     * @return voltage
     */
    public Voltage getVoltage()
    {
        return master.getMotorVoltage().getValue();
    }

    /*
    public Angle getDesiredPosition()
    {
        return Rotations.of(this.desiredPosition);
    }
    */

    
    /*
    public void setPosition(Angle position)
    {
        master.setPosition(position);
    }
    */

    public void setVelocity(AngularVelocity velocity)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hopper");
            return;
        }
        master.setControl(new VelocityVoltage(velocity));
    }

    /**
     * @param power variable between [-1, 1]
     */
    public void setDutyCycle(double power)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hopper");
            return;
        }
        master.setControl(new DutyCycleOut(power));
    }

    /*
    //sets the target & moves there
    public void setDesiredPosition(Angle target)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hopper");
            return;
        }
        this.desiredPosition = target.in(Rotations);
        master.setControl(new MotionMagicVoltage(target));
    }
    */

    public void setVoltage(Voltage voltage)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Hopper");
            return;
        }
        master.setVoltage(voltage.in(Volts));
    }



    public boolean isStalling()
    {
        return Math.abs(master.getStatorCurrent().getValueAsDouble()) >= Constants.Hopper.STALLING_CURRENT;
    }

    /*
    public boolean atPosition()
    {
        return Math.abs(this.desiredPosition - getPosition().in(Rotations)) <= Constants.EPSILON;
    }
    */

    
    @Override
    public void periodic()
    {
        if (isSimulated())
        {
            TalonFXSimState simState = master.getSimState();

            // set the supply voltage of the TalonFX
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            Voltage motorVoltage = simState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            sim.setInputVoltage(motorVoltage.in(Volts));
            sim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            simState.setRawRotorPosition(sim.getPositionMeters() * Constants.Hopper.GEAR_RATIO);
            simState.setRotorVelocity(sim.getVelocityMetersPerSecond() * Constants.Hopper.GEAR_RATIO);
        }
    }
    
    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.HOPPER_INDEX) == SubsystemStatus.Simulated;
    }
    
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.HOPPER_INDEX) == SubsystemStatus.Disabled;
    }
    
    /*
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->setVoltage(v),
            (SysIdRoutineLog l)->l
                .motor("Hopper")
                .voltage(getVoltage())
                .angularPosition(master.getPosition().getValue())
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


    public static Hopper getInstance() 
    {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }
}
