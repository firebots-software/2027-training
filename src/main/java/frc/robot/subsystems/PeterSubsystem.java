package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class PeterSubsystem extends SubsystemBase {
  private static PeterSubsystem instance;

  private DigitalInput noteSensor;
  private LoggedTalonFX shooter1, shooter2, preShooterMotor, intakeMotor;
  private MotionMagicConfigs mmcPreShooter;

  public PeterSubsystem() {
    // === Shooter Setup ===
    shooter2 = new LoggedTalonFX("shooter_left", Constants.Pooer.SHOOTER.SHOOTER_1.PORT, Constants.Pooer.CANBUS_NAME);
    shooter1 = new LoggedTalonFX("shooter_right", Constants.Pooer.SHOOTER.SHOOTER_2.PORT, Constants.Pooer.CANBUS_NAME);

    MotorOutputConfigs mocshooter = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
    shooter1.getConfigurator().apply(mocshooter);
    shooter2.getConfigurator().apply(mocshooter);

    // TODO: Tune KP for the shooter. Feedforward (KV) is provided.
    Slot0Configs s0c = new Slot0Configs().withKP(0.0).withKI(0).withKD(0).withKG(0).withKV(0.1185).withKA(0);
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Constants.Pooer.SHOOTER.SHOOTER_1.STATOR_CURRENT_LIMIT_AMPS);

    shooter1.getConfigurator().apply(s0c);
    shooter2.getConfigurator().apply(s0c);
    shooter1.getConfigurator().apply(clc);
    shooter2.getConfigurator().apply(clc);

    // === PreShooter Setup ===
    preShooterMotor = new LoggedTalonFX("preshooter", Constants.Pooer.SHOOTER.PRESHOOTER.PORT,
        Constants.Pooer.CANBUS_NAME);
    MotorOutputConfigs mocpreShooterMotor = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    preShooterMotor.getConfigurator().apply(mocpreShooterMotor);

    mmcPreShooter = new MotionMagicConfigs();
    mmcPreShooter.MotionMagicCruiseVelocity = 80;
    mmcPreShooter.MotionMagicAcceleration = 160;
    mmcPreShooter.MotionMagicJerk = 1600;
    preShooterMotor.getConfigurator().apply(mmcPreShooter);

    // TODO: Tune KP for the preshooter. Feedforward (KV) is provided.
    Slot0Configs preshooterPID = new Slot0Configs().withKP(0.0).withKV(1);
    preShooterMotor.getConfigurator().apply(preshooterPID);
    preShooterMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Constants.Pooer.SHOOTER.PRESHOOTER.STATOR_CURRENT_LIMIT_AMPS));

    // === Intake Setup ===
    // TODO: Tune KP for the intake.
    Slot0Configs intakePid = new Slot0Configs().withKP(0.0).withKI(0).withKD(0).withKG(0).withKV(0).withKA(0);
    intakeMotor = new LoggedTalonFX("intake", Constants.Pooer.SHOOTER.INTAKE.PORT, Constants.Pooer.CANBUS_NAME);
    intakeMotor.getConfigurator().apply(intakePid);
    intakeMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Constants.Pooer.SHOOTER.INTAKE.STATOR_CURRENT_LIMIT_AMPS));
    MotorOutputConfigs mocintakeMotor = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    intakeMotor.getConfigurator().apply(mocintakeMotor);

    // === Sensor Setup ===
    noteSensor = new DigitalInput(Constants.Pooer.NOTE_DETECTOR_PORT);
  }

  public static PeterSubsystem getInstance() {
    if (instance == null) {
      instance = new PeterSubsystem();
    }
    return instance;
  }

  // TODO: Design and implement the methods required to run the intake,
  // pre-shooter, and shooter.
  // Consider what commands will need (e.g., setting speeds, checking if the
  // shooter is up to speed, checking the IR sensor).

  public void runIntake(double speed) {
    VelocityVoltage m_VelocityVoltage = new VelocityVoltage(speed * Constants.Pooer.SHOOTER.INTAKE.GEAR_RATIO);
    intakeMotor.setControl(m_VelocityVoltage);
  }

  public void runShooter1(double speed) {
    VelocityVoltage m1_VelocityVoltage = new VelocityVoltage(speed * Constants.Pooer.SHOOTER.SHOOTER_1.GEAR_RATIO);
    shooter1.setControl(m1_VelocityVoltage);
  }

  public void runShooter2(double speed) {
    VelocityVoltage m2_VelocityVoltage = new VelocityVoltage(speed * Constants.Pooer.SHOOTER.SHOOTER_2.GEAR_RATIO);
    shooter2.setControl(m2_VelocityVoltage);
  }

  public void runShooter(double speed1, double speed2) {
    runShooter1(speed1);
    runShooter2(speed2);
  }

  public void runPreShooter(double speed) {
    VelocityVoltage m_VelocityVoltage = new VelocityVoltage(speed * Constants.Pooer.SHOOTER.PRESHOOTER.GEAR_RATIO);
    preShooterMotor.setControl(m_VelocityVoltage);
  }

  public boolean isNotePresent() {
    return !noteSensor.get();
  }

  public boolean isShooterAtTarget() {
    return Math.abs(shooter1.getVelocity().getValueAsDouble()
        - Constants.Pooer.SHOOTER.SHOOTER_1.SPEED_RPS * Constants.Pooer.SHOOTER.SHOOTER_2.GEAR_RATIO) < 10;
  }

  @Override
  public void periodic() {
    // TODO: Add logging
  }
}