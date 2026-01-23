package frc.robot.subsystems;

import java.nio.file.attribute.PosixFileAttributeView;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * 
 */
public class Hood extends SubsystemBase
{
    private static Hood instance;

    private static TalonFX motor;

    private double desiredPosition; // rotations
    
    private final SingleJointedArmSim motorSimModel = new SingleJointedArmSim(
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1), 0.001, Constants.Hood.GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1)).getGearbox(),
        Constants.Hood.GEAR_RATIO,
        Constants.Hood.MOMENT_OF_INERTIA,
        Constants.Hood.HOOD_LENGTH,
        Units.degreesToRadians(Constants.Hood.HOOD_MIN_ANGLE),
        Units.degreesToRadians(Constants.Hood.HOOD_MAX_ANGLE),
        true,
        Units.degreesToRadians(10.0)
        // no std devs -> no noise simulated
        );



    private Hood()
    {
        motor = new TalonFX(Constants.Hood.MOTOR_ID);

        config();

        if (Robot.isSimulation())
        {
            TalonFXSimState simState = motor.getSimState();
            simState.Orientation = Constants.Hood.MECHANICAL_ORIENTATION;
            simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
    }

    private void config()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        if (Robot.isReal())
        {
            config.CurrentLimits.StatorCurrentLimit = Constants.Hood.STATOR_CURRENT_LIMIT;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            
            config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        config.Feedback.SensorToMechanismRatio = Constants.Hood.GEAR_RATIO;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.MM_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = Constants.Hood.MM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = Constants.Hood.MM_JERK;

        config.MotorOutput.Inverted = Constants.Hood.INVERTED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = Constants.Hood.KP;
        config.Slot0.kI = Constants.Hood.KI;
        config.Slot0.kD = Constants.Hood.KD;
        config.Slot0.kG = Constants.Hood.KG;
        config.Slot0.kV = Constants.Hood.KV;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        motor.getConfigurator().apply(config);

    }

    public void moveToPosition(Angle desiredPosition)
    {
        this.desiredPosition = desiredPosition.in(Rotations);
        motor.setControl(new MotionMagicVoltage(desiredPosition));
    }

    public Angle getPosition()
    {
        return motor.getPosition().getValue();
    }
    
    public Voltage getVoltage()
    {
        return motor.getMotorVoltage().getValue();
    }
    
    public AngularVelocity getVelocity()
    {
        return motor.getVelocity().getValue();
    }
    
    public boolean readyToShoot ()
    {
        return Math.abs(motor.getPosition().getValue().in(Rotation) - desiredPosition) < Constants.EPSILON;
    }
    
    public Angle getDesiredPosition()
    {
        return Rotations.of(desiredPosition);
    }

    @Override
    public void simulationPeriodic()
    {
        TalonFXSimState simState = motor.getSimState();

        // set the supply voltage of the TalonFX
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        Voltage motorVoltage = simState.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        simState.setRawRotorPosition(Units.radiansToRotations(motorSimModel.getAngleRads()) * Constants.Hood.GEAR_RATIO);
        simState.setRotorVelocity(Units.radiansToRotations(motorSimModel.getVelocityRadPerSec()) * Constants.Hood.GEAR_RATIO);
    }

    
    private SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism((Voltage v)->motor.setControl(new VoltageOut(v)),
            (SysIdRoutineLog l)->l
                .motor("Hood")
                .voltage(getVoltage())
                .angularPosition(getPosition())
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
    

    public static Hood getInstance()
    {
        if (instance == null) instance = new Hood();
        return instance;
    }
}
