package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase {

        private final LoggedTalonFX warmup1, warmup2, warmup3, shooter, hood;
        private final CANcoder hoodEncoder;

        /*
         * make instance variables for necessary closed loop control requests:
         * VelocityVoltage, PositionVoltage. Initialize them with value 0.0.
         * make instance variables of type double for the target roller speed and target
         * hood angle
         */

        private final VelocityVoltage m_velocityVoltageRequest = new VelocityVoltage(0.0);
        private final PositionVoltage m_positionVoltageRequest = new PositionVoltage(0.0);
        private double target_roller_speed, target_hood_angle;

        public ShooterSubsystem() {
                // The canbus is a communication system that can connect devices like the
                // roboRIO, pdh, and motors.
                CANBus canbus = Constants.Swerve.CAN_BUS;

                // LoggedTalonFX is our version of the existing TalonFX class, which
                // automatically logs some motor information
                warmup1 = new LoggedTalonFX("ShooterWarmup1", Constants.Shooter.Rollers.WARMUP_1_ID, canbus);
                warmup2 = new LoggedTalonFX("ShooterWarmup2", Constants.Shooter.Rollers.WARMUP_2_ID, canbus);
                warmup3 = new LoggedTalonFX("ShooterWarmup3", Constants.Shooter.Rollers.WARMUP_3_ID, canbus);

                shooter = warmup3; // warmup1 and warmup2 will be set as "followers" of the "lead" warmup3, so
                                   // later we can just reference the `shooter` variable

                // Initialize the hood motor just like the warmup motors. You can find the
                // necessary ID in Constants.
                hood = new LoggedTalonFX("Hood", Constants.Shooter.Hood.HOOD_ID, canbus);

                // Create a variable of type Slot0Configs called rollersSlot0Configs, and
                // initialize it with the pid and feedforward gains found in Constants
                Slot0Configs rollersSlot0Configs = new Slot0Configs()
                                .withKP(Constants.Shooter.Rollers.KP)
                                .withKI(Constants.Shooter.Rollers.KI)
                                .withKD(Constants.Shooter.Rollers.KD)
                                .withKV(Constants.Shooter.Rollers.KV)
                                .withKS(Constants.Shooter.Rollers.KS);

                // Create a variable of type CurrentLimitsConfigs called rollersClConfigs, and
                // initialize it with the stator and supply limits found in Constants
                CurrentLimitsConfigs rollersClConfigs = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Constants.Shooter.Rollers.STATOR_CURRENT_LIMIT)
                                .withSupplyCurrentLimit(Constants.Shooter.Rollers.SUPPLY_CURRENT_LIMIT);

                // Create a variable of type MotorOutputConifigs called rollersOutputConfigs,
                // and initialize it to treat clockwise as positive, and neutral mode as coast
                MotorOutputConfigs rollersOutputConfigs = new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast);

                // Set the Slot0, CurrentLimits, and MotorOutput parameters of `rollersConfig`
                // to the configs you just created.
                TalonFXConfiguration rollersConfig = new TalonFXConfiguration()
                                .withSlot0(rollersSlot0Configs)
                                .withCurrentLimits(rollersClConfigs)
                                .withMotorOutput(rollersOutputConfigs);

                warmup1.getConfigurator().apply(rollersConfig);
                // I just applied the configuration that you just created to warmup1. Apply it
                // to warmup2 and warmup3 as well.
                warmup2.getConfigurator().apply(rollersConfig);
                warmup3.getConfigurator().apply(rollersConfig);

                Follower follower = new Follower(Constants.Shooter.Rollers.WARMUP_3_ID, MotorAlignmentValue.Aligned);
                // use the `setControl` method of warmup1 and 2 to configure them to use this
                // follower configuration.
                warmup1.setControl(follower);
                warmup2.setControl(follower);

                hoodEncoder = new CANcoder(Constants.Shooter.Hood.ENCODER_PORT, Constants.Swerve.CAN_BUS);

                // Create a variable of type Slot0Configs called hoodSlot0Configs, and
                // initialize it with the pid and feedforward gains found in Constants
                Slot0Configs hoodSlot0Configs = new Slot0Configs()
                                .withKP(Constants.Shooter.Hood.KP)
                                .withKI(Constants.Shooter.Hood.KI)
                                .withKD(Constants.Shooter.Hood.KD)
                                .withKV(Constants.Shooter.Hood.KV)
                                .withKS(Constants.Shooter.Hood.KS)
                                .withKG(Constants.Shooter.Hood.KG);

                // Create a variable of type CurrentLimitsConfigs called hoodClConfigs, and
                // initialize it with the stator and supply limits found in Constants
                CurrentLimitsConfigs hoodClConfigs = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Constants.Shooter.Hood.STATOR_CURRENT_LIMIT)
                                .withSupplyCurrentLimit(Constants.Shooter.Hood.SUPPLY_CURRENT_LIMIT);

                // Create a variable of type MotorOutputConifigs called hoodOutputConfigs, and
                // initialize it to treat counterclockwise as positive, and neutral mode as
                // coast
                MotorOutputConfigs hoodOutputConfigs = new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast);

                FeedbackConfigs hoodFeedbackConfigs = new FeedbackConfigs()
                                .withFeedbackRemoteSensorID(hoodEncoder.getDeviceID())
                                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                .withSensorToMechanismRatio(Constants.Shooter.Hood.ENCODER_ROTS_PER_HOOD_ROT)
                                .withRotorToSensorRatio(Constants.Shooter.Hood.MOTOR_ROTS_PER_ENCODER_ROT);

                // Create a TalonFXConfiguration called hoodConfig
                // Set the Slot0, CurrentLimits, MotorOutput, and Feedback parameters to the
                // device configs above
                TalonFXConfiguration hoodConfig = new TalonFXConfiguration()
                                .withSlot0(hoodSlot0Configs)
                                .withCurrentLimits(hoodClConfigs)
                                .withMotorOutput(hoodOutputConfigs)
                                .withFeedback(hoodFeedbackConfigs);

                // Apply the created configuration to the hood motor
                hood.getConfigurator().apply(hoodConfig);

                MagnetSensorConfigs hoodCANcoderConfig = new CANcoderConfiguration().MagnetSensor
                                .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                                .withMagnetOffset(Rotations.of(Constants.Shooter.Hood.ENCODER_OFFSET));

                // Apply this config to the `hoodEncoder`. Notice how applying a device
                // configuration is similar between motors and other devices
                hoodEncoder.getConfigurator().apply(hoodCANcoderConfig);

        }
}
