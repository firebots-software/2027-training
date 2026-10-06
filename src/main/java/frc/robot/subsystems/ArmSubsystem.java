package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.util.LoggedTalonFX;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance;

  private LoggedTalonFX rt, rb, lt, lb;
  private LoggedTalonFX master;
  private DutyCycleEncoder revEncoder;
  private ArmFeedforward armff;
  private MotionMagicConfigs mmc;

  private boolean initialized = false;
  private double armHorizontalOffset;

  public ArmSubsystem() {
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Arm.ARM_STATOR_CURRENT_LIMIT_AMPS);
    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    
    // TODO: Tune the Arm KP value. Feedforward is provided below.
    Slot0Configs s0c = new Slot0Configs().withKP(0.0).withKI(0).withKD(0);
    
    armff = new ArmFeedforward(Constants.Arm.ARMFF_KS, Constants.Arm.ARMFF_KG, Constants.Arm.ARMFF_KV);

    rt = new LoggedTalonFX("ArmRightTop", Constants.Arm.RT_PORT, Constants.Arm.CANBUS_NAME);
    rb = new LoggedTalonFX("ArmRightBottom", Constants.Arm.RB_PORT, Constants.Arm.CANBUS_NAME);
    lt = new LoggedTalonFX("ArmLefttop", Constants.Arm.LT_PORT, Constants.Arm.CANBUS_NAME);
    lb = new LoggedTalonFX("ArmLeftBottom", Constants.Arm.LB_PORT, Constants.Arm.CANBUS_NAME);

    Follower follower = new Follower(Constants.Arm.LT_PORT, MotorAlignmentValue.Aligned);
    Follower invertedFollower = new Follower(Constants.Arm.LT_PORT, MotorAlignmentValue.Opposed);
    rt.setControl(invertedFollower);
    rb.setControl(invertedFollower);
    lb.setControl(follower);

    TalonFXConfigurator rtConfig = rt.getConfigurator();
    TalonFXConfigurator rbConfig = rb.getConfigurator();
    TalonFXConfigurator ltConfig = lt.getConfigurator();
    TalonFXConfigurator lbConfig = lb.getConfigurator();

    rtConfig.apply(moc); rbConfig.apply(moc); ltConfig.apply(moc); lbConfig.apply(moc);
    rtConfig.apply(clc); rbConfig.apply(clc); ltConfig.apply(clc); lbConfig.apply(clc);

    master = lt;
    TalonFXConfigurator masterConfigurator = master.getConfigurator();
    masterConfigurator.apply(s0c);

    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = Constants.Arm.MOTIONMAGIC_KV * Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
    mmc.MotionMagicAcceleration = Constants.Arm.MOTIONMAGIC_KA * Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
    masterConfigurator.apply(mmc);

    revEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);

    new Thread(() -> {
      try {
        do {
          Thread.sleep(250);
        } while (!revEncoder.isConnected());
        // Uses the Absolute Encoder to set the position of the Master motor, so that when the Master reads 0,
        // it represents the arm being at horizontal.
        master.setPosition((getAbsolutePosition()) * Constants.Arm.INTEGRATED_ABSOLUTE_CONVERSION_FACTOR);
        initialized = true;
        armHorizontalOffset = Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET / Constants.Arm.ABSOLUTE_ARM_CONVERSION_FACTOR;
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }).start();
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) { instance = new ArmSubsystem(); }
    return instance;
  }

  private double getAbsolutePosition() {
    return (revEncoder.get() - Constants.Arm.ABSOLUTE_ENCODER_HORIZONTAL + Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET + 1d) % 1;
  }

  // TODO: Design and implement the methods required to control the arm's position.
  // Consider what your arm commands will need to function properly (e.g., setting angles, reading current angles, checking tolerances).

  public void moveToPosition(double position) {
    master.setControl(new MotionMagicVoltage(position));
  }

  public void moveToIntake() {
    master.setControl(new MotionMagicVoltage(Constants.Arm.INTAKE_ANGLE));
  }

  public void goToNeutral() {
    master.setControl(new MotionMagicVoltage(Constants.Arm.DEFAULT_ARM_ANGLE));

  }

  @Override
  public void periodic() {
    // TODO: Execute your control logic and any necessary logic here
  }
}