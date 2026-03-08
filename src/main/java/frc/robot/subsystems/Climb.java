package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SubsystemStatus;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class Climb extends SubsystemBase 
{
    private static Climb instance;

    private TalonFX climbWheels;
    private TalonFX spooling;

    // this doesn't work anymore since its not an elevator and idk how you would sim it
    private DCMotorSim spoolSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Climb.SPOOLING_GEAR_RATIO),
            DCMotor.getKrakenX60(1));


    private DCMotorSim climbWheelsSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),0.001, Constants.Climb.CLIMBWHEELS_GEAR_RATIO),
        DCMotor.getKrakenX60(1));

    private Climb() 
    {
        climbWheels = new TalonFX(Constants.Climb.CLIMBWHEELS_ID, Constants.CAN_SUPERSTRUCTURE);
        spooling = new TalonFX(Constants.Climb.SPOOLING_ID, Constants.CAN_SUPERSTRUCTURE);

        config();
        
        if (isSimulated())
        {
            TalonFXSimState elevatorSimState = climbWheels.getSimState();
            elevatorSimState.Orientation = Constants.Climb.CLIMBWHEELS_MECHANICAL_ORIENTATION;
            elevatorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
            
            TalonFXSimState climbSimState = spooling.getSimState();
            climbSimState.Orientation = Constants.Climb.SPOOLING_MECHANICAL_ORIENTATION;
            climbSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    private void config() 
    {
        climbWheels.clearStickyFaults();
        spooling.clearStickyFaults();

        TalonFXConfiguration climbWheelsConfig = new TalonFXConfiguration();

        climbWheelsConfig.MotorOutput.Inverted = Constants.Climb.CLIMBWHEELS_INVERTED;

        climbWheelsConfig.Feedback.SensorToMechanismRatio = Constants.Climb.CLIMBWHEELS_GEAR_RATIO;
        
        climbWheelsConfig.CurrentLimits.StatorCurrentLimit = Constants.Climb.CLIMBWHEELS_STATOR_CURRENT_LIMIT;
        climbWheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        climbWheelsConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.CLIMBWHEELS_SUPPLY_CURRENT_LIMIT;
        climbWheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        climbWheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climbWheelsConfig.Slot0.kP = Constants.Climb.KP_CLIMBWHEELS;
        climbWheelsConfig.Slot0.kI = Constants.Climb.KI_CLIMBWHEELS;
        climbWheelsConfig.Slot0.kD = Constants.Climb.KD_CLIMBWHEELS;

        climbWheels.getConfigurator().apply(climbWheelsConfig);

        TalonFXConfiguration spoolingConfig = new TalonFXConfiguration();

        spoolingConfig.MotorOutput.Inverted = Constants.Climb.SPOOLING_INVERTED;

        spoolingConfig.Feedback.SensorToMechanismRatio = Constants.Climb.SPOOLING_GEAR_RATIO;
        
        spoolingConfig.CurrentLimits.StatorCurrentLimit = Constants.Climb.SPOOLING_STATOR_CURRENT_LIMIT;
        spoolingConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        spoolingConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.SPOOLING_SUPPLY_CURRENT_LIMIT;
        spoolingConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        spoolingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        spoolingConfig.Slot0.kP = Constants.Climb.KP_SPOOLING;
        spoolingConfig.Slot0.kI = Constants.Climb.KI_SPOOLING;
        spoolingConfig.Slot0.kD = Constants.Climb.KD_SPOOLING;

        spooling.getConfigurator().apply(spoolingConfig);
    }

    public void setClimbWheelsVelocity(AngularVelocity velocity) 
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        climbWheels.setControl(new VelocityVoltage(velocity));
    }

    public AngularVelocity getClimbWheelsVelocity() 
    {
        return climbWheels.getVelocity().getValue();
    }

    public Voltage getClimbWheelsVoltage()
    {
        return climbWheels.getMotorVoltage().getValue();
    }

    public Angle getClimbWheelsPosition()
    {
        return climbWheels.getPosition().getValue();
    }

    public void setClimbWheelsVoltage(Voltage v)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        climbWheels.setControl(new VoltageOut(v));
    }

    public boolean isSpoolingStalling()
    {
        return Math.abs(spooling.getStatorCurrent().getValueAsDouble()) >= Constants.Climb.SPOOLING_STALLING_CURRENT;
    }

    public Voltage getSpoolingVoltage()
    {
        return spooling.getMotorVoltage().getValue();
    }
    
    public Angle getSpoolingPosition()
    {
        return spooling.getPosition().getValue();
    }

    public void setSpoolingVoltage(Voltage v)
    {
        if (isDisabled())
        {
            System.out.println("Quashing input to Climb");
            return;
        }
        spooling.setControl(new VoltageOut(v));
    }
    
    @Override
    public void periodic()
    {
        if (isSimulated())
        {
            TalonFXSimState climbWheelsSimState = climbWheels.getSimState();

            // set the supply voltage of the TalonFX
            climbWheelsSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            Voltage climbWheelsMotorVoltage = climbWheelsSimState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            climbWheelsSim.setInputVoltage(climbWheelsMotorVoltage.in(Volts));
            climbWheelsSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            climbWheelsSimState.setRawRotorPosition(climbWheelsSim.getAngularPositionRotations() * Constants.Climb.CLIMBWHEELS_GEAR_RATIO);
            climbWheelsSimState
                    .setRotorVelocity(climbWheelsSim.getAngularVelocity().in(RotationsPerSecond) * Constants.Climb.CLIMBWHEELS_GEAR_RATIO);


            TalonFXSimState spoolingSimState = spooling.getSimState();

            // set the supply voltage of the TalonFX
            spoolingSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            // get the motor voltage of the TalonFX
            Voltage climbMotorVoltage = spoolingSimState.getMotorVoltageMeasure();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            spoolSim.setInputVoltage(climbMotorVoltage.in(Volts));
            spoolSim.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            spoolingSimState.setRawRotorPosition(
                    spoolSim.getAngularPosition().in(Rotations) * Constants.Climb.SPOOLING_GEAR_RATIO);
            spoolingSimState.setRotorVelocity(
                    spoolSim.getAngularVelocity().in(Rotations.per(Second)) * Constants.Climb.SPOOLING_GEAR_RATIO);
        }
    }

    private boolean isSimulated ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.CLIMB_INDEX) == SubsystemStatus.Simulated;
    }
    
    private boolean isDisabled ()
    {
        return Robot.instance.robotContainer.getStatus(RobotContainer.CLIMB_INDEX) == SubsystemStatus.Disabled;
    }
    
    //returns the instance of the subsystem
    public static Climb getInstance() 
    {
        if (instance == null) 
        {
            instance = new Climb();
        }
        return instance;
    }
}