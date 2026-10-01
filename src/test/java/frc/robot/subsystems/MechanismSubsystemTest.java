package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class MechanismSubsystemTest {
  private static ArmSubsystem arm;
  private static PeterSubsystem peter;
  private static LoggedTalonFX master;
  private static LoggedTalonFX shooter1;
  private static LoggedTalonFX shooter2;
  private static DutyCycleEncoderSim encoder;

  @BeforeAll
  static void createHardwareSimulation() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    arm = new ArmSubsystem();
    peter = new PeterSubsystem();
    master = hardwareField(arm, "master", LoggedTalonFX.class);
    shooter1 = hardwareField(peter, "shooter1", LoggedTalonFX.class);
    shooter2 = hardwareField(peter, "shooter2", LoggedTalonFX.class);
    encoder = new DutyCycleEncoderSim(hardwareField(arm, "revEncoder", DutyCycleEncoder.class));
    master.getSimState().setSupplyVoltage(12);
    shooter1.getSimState().setSupplyVoltage(12);
    shooter2.getSimState().setSupplyVoltage(12);
  }

  @BeforeEach
  void calibrateArmAndStopShooter() throws InterruptedException {
    encoder.set(Constants.Arm.ABSOLUTE_ENCODER_HORIZONTAL);
    encoder.setConnected(true);
    await(() -> {
      arm.reset();
      return arm.isInitialized();
    }, "Simulated arm should calibrate from the connected absolute encoder");
    peter.stopShooter();
  }

  @AfterAll
  static void closeHardwareSimulation() throws Exception {
    CommandScheduler.getInstance().unregisterSubsystem(arm, peter);
    closeHardware(arm);
    closeHardware(peter);
  }

  @Test
  void individuallyNamedShootersUseTheirCorrespondingCanIds() {
    assertEquals(Constants.Pooer.SHOOTER.SHOOTER_1.PORT, shooter1.getDeviceID());
    assertEquals(Constants.Pooer.SHOOTER.SHOOTER_2.PORT, shooter2.getDeviceID());
  }

  @Test
  void readinessRequiresBothWheelsAtTheirRequestedSpeeds() throws InterruptedException {
    peter.runShooter(40, 70);
    simulateShooterSpeeds(40, 0);
    assertFalse(peter.isShooterReady(), "A stopped second wheel must prevent shooting");
    assertFalse(peter.isShooterAtTarget());

    simulateShooterSpeeds(40, 70);
    assertTrue(peter.isShooterReady(), "Readiness must use the requested speeds, not defaults");
    assertTrue(peter.isShooterAtTarget());

    peter.stopShooter();
    assertFalse(peter.isShooterReady(), "Residual measured speed cannot make a stopped shooter ready");
    assertFalse(peter.isShooterAtTarget());
  }

  @Test
  void oneCommandedWheelCannotMakeTheShooterReady() throws InterruptedException {
    peter.runShooter1(40);
    simulateShooterSpeeds(40, 70);
    assertFalse(peter.isShooterReady());
    assertFalse(peter.isShooterAtTarget());
  }

  @Test
  void angleAndPositionSettersSurviveSubsequentPeriodicUpdates() {
    arm.setTargetDegrees(20);
    arm.periodic();
    arm.setAngle(45);
    arm.periodic();
    assertArmRequestDegrees(45);

    arm.setPosition(60);
    arm.periodic();
    assertArmRequestDegrees(60);
  }

  @Test
  void failedCalibrationCancelsThePreviousMotorRequest() {
    arm.setPosition(45);
    arm.periodic();
    assertArmRequestDegrees(45);

    encoder.setConnected(false);
    arm.reset();
    assertFalse(arm.isInitialized());
    assertInstanceOf(NeutralOut.class, master.getAppliedControl());

    arm.periodic();
    assertFalse(arm.isInitialized());
    assertInstanceOf(NeutralOut.class, master.getAppliedControl(),
        "Periodic must not restore a position request before recalibration");
  }

  private static void simulateShooterSpeeds(double first, double second) throws InterruptedException {
    double firstRotorRps = first * Constants.Pooer.SHOOTER.SHOOTER_1.GEAR_RATIO;
    double secondRotorRps = second * Constants.Pooer.SHOOTER.SHOOTER_2.GEAR_RATIO;
    assertTrue(shooter1.getSimState().setRotorVelocity(firstRotorRps).isOK());
    assertTrue(shooter2.getSimState().setRotorVelocity(secondRotorRps).isOK());
    await(() -> Math.abs(shooter1.getVelocity().refresh().getValueAsDouble() - firstRotorRps) < 0.1
        && Math.abs(shooter2.getVelocity().refresh().getValueAsDouble() - secondRotorRps) < 0.1,
        "CTRE simulation should publish both injected rotor velocities");
  }

  private static void assertArmRequestDegrees(double expectedDegrees) {
    PositionVoltage request = assertInstanceOf(PositionVoltage.class, master.getAppliedControl());
    double requestedDegrees = (request.Position / Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR
        - Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET / Constants.Arm.ABSOLUTE_ARM_CONVERSION_FACTOR) * 360;
    assertEquals(expectedDegrees, requestedDegrees, 1e-6);
  }

  private static void await(BooleanSupplier condition, String message) throws InterruptedException {
    long deadline = System.nanoTime() + 2_000_000_000L;
    while (!condition.getAsBoolean()) {
      if (System.nanoTime() >= deadline) {
        fail(message);
      }
      Thread.sleep(10);
    }
  }

  private static <T> T hardwareField(Object owner, String name, Class<T> type) throws Exception {
    Field field = owner.getClass().getDeclaredField(name);
    field.setAccessible(true);
    return type.cast(field.get(owner));
  }

  private static void closeHardware(Object subsystem) throws Exception {
    Set<Object> closed = Collections.newSetFromMap(new IdentityHashMap<>());
    for (Field field : subsystem.getClass().getDeclaredFields()) {
      if (Modifier.isStatic(field.getModifiers())) {
        continue;
      }
      field.setAccessible(true);
      Object hardware = field.get(subsystem);
      if (hardware instanceof AutoCloseable closeable && closed.add(hardware)) {
        closeable.close();
      }
    }
  }
}
