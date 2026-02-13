package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

public class Modules {
    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(Constants.Swerve.kCANBus.getName())
            .withPigeon2Id(Constants.Swerve.kPigeonId)
            .withPigeon2Configs(Constants.Swerve.pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(Constants.Swerve.kDriveGearRatio)
            .withSteerMotorGearRatio(Constants.Swerve.kSteerGearRatio)
            .withCouplingGearRatio(Constants.Swerve.kCoupleRatio)
            .withWheelRadius(Constants.Swerve.kWheelRadius)
            .withSteerMotorGains(Constants.Swerve.steerGains)
            .withDriveMotorGains(Constants.Swerve.driveGains)
            .withSteerMotorClosedLoopOutput(Constants.Swerve.kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(Constants.Swerve.kDriveClosedLoopOutput)
            .withSlipCurrent(Constants.Swerve.kSlipCurrent)
            .withSpeedAt12Volts(Constants.Swerve.kSpeedAt12Volts)
            .withDriveMotorType(Constants.Swerve.kDriveMotorType)
            .withSteerMotorType(Constants.Swerve.kSteerMotorType)
            .withFeedbackSource(Constants.Swerve.kSteerFeedbackType)
            .withDriveMotorInitialConfigs(Constants.Swerve.driveInitialConfigs)
            .withSteerMotorInitialConfigs(Constants.Swerve.steerInitialConfigs)
            .withEncoderInitialConfigs(Constants.Swerve.encoderInitialConfigs)
            .withSteerInertia(Constants.Swerve.kSteerInertia)
            .withDriveInertia(Constants.Swerve.kDriveInertia)
            .withSteerFrictionVoltage(Constants.Swerve.kSteerFrictionVoltage)
            .withDriveFrictionVoltage(Constants.Swerve.kDriveFrictionVoltage);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
            .createModuleConstants(
                    Constants.Swerve.kFrontLeftSteerMotorId,
                    Constants.Swerve.kFrontLeftDriveMotorId,
                    Constants.Swerve.kFrontLeftEncoderId, Constants.Swerve.kFrontLeftEncoderOffset,
                    Constants.Swerve.kFrontLeftXPos, Constants.Swerve.kFrontLeftYPos,
                    Constants.Swerve.kInvertLeftSide,
                    Constants.Swerve.kFrontLeftSteerMotorInverted,
                    Constants.Swerve.kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
            .createModuleConstants(
                    Constants.Swerve.kFrontRightSteerMotorId,
                    Constants.Swerve.kFrontRightDriveMotorId,
                    Constants.Swerve.kFrontRightEncoderId,
                    Constants.Swerve.kFrontRightEncoderOffset,
                    Constants.Swerve.kFrontRightXPos, Constants.Swerve.kFrontRightYPos,
                    Constants.Swerve.kInvertRightSide,
                    Constants.Swerve.kFrontRightSteerMotorInverted,
                    Constants.Swerve.kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
            .createModuleConstants(
                    Constants.Swerve.kBackLeftSteerMotorId, Constants.Swerve.kBackLeftDriveMotorId,
                    Constants.Swerve.kBackLeftEncoderId, Constants.Swerve.kBackLeftEncoderOffset,
                    Constants.Swerve.kBackLeftXPos, Constants.Swerve.kBackLeftYPos,
                    Constants.Swerve.kInvertLeftSide,
                    Constants.Swerve.kBackLeftSteerMotorInverted,
                    Constants.Swerve.kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
            .createModuleConstants(
                    Constants.Swerve.kBackRightSteerMotorId,
                    Constants.Swerve.kBackRightDriveMotorId,
                    Constants.Swerve.kBackRightEncoderId, Constants.Swerve.kBackRightEncoderOffset,
                    Constants.Swerve.kBackRightXPos, Constants.Swerve.kBackRightYPos,
                    Constants.Swerve.kInvertRightSide,
                    Constants.Swerve.kBackRightSteerMotorInverted,
                    Constants.Swerve.kBackRightEncoderInverted);

    /**
     * Creates a Drivetrain instance.
     * This should only be called once in your robot program,.
     */
    public static Drivetrain createDrivetrain() {
        return new Drivetrain(
                DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }

    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected
     * device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct
         * the devices themselves. If they need the devices, they can access them
         * through
         * getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param modules             Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct
         * the devices themselves. If they need the devices, they can access them
         * through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct
         * the devices themselves. If they need the devices, they can access them
         * through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve
         *                                  drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz
         *                                  on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry
         *                                  calculation
         *                                  in the form [x, y, theta]ᵀ, with units in
         *                                  meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision
         *                                  calculation
         *                                  in the form [x, y, theta]ᵀ, with units in
         *                                  meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency,
                    odometryStandardDeviation, visionStandardDeviation, modules);
        }
    }
}