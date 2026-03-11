package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor = new TalonFX(RobotMap.TurretCanID);
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(75, 0, 0.25, RPM.of(2000), RotationsPerSecondPerSecond.of(20))
          // Configure Motor and Mechanism properties
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(45.45, 1)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          // Setup Telemetry
          .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
          // Power Optimization
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));
  private final SmartMotorController turretSMC =
      new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

  private final PivotConfig turretConfig =
      new PivotConfig(turretSMC)
          .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
          .withWrapping(
              edu.wpi.first.units.Units.Rotations.of(-0.5),
              edu.wpi.first.units.Units.Rotations.of(0.5)) // Wrapping enabled bc the pivot can spin
          // infinitely
          .withHardLimit(
              Degrees.of(-99.5),
              Degrees.of(99.5)) // Hard limit bc wiring prevents infinite spinning
          .withTelemetry("TurretMech", TelemetryVerbosity.HIGH) // Telemetry
          .withMOI(Meters.of(0.25), Pounds.of(4)); // MOI Calculation

  private final Pivot turret = new Pivot(turretConfig);
  // Variables based on teeth given by machanical.
  private final double t_teeth = 250;
  private final double e1_teeth = (0.1) * t_teeth;
  private final double e2_teeth = 0.024 * t_teeth;
  // offest until I find the actual one.
  private final double e1_offset = 0.0;
  private final double e2_offset = 0.0;

  // n is the number of integer rotations and E is the encoders in degrees interms of data given.

  // private final EasyCRTConfig easyCRTConfig;

  public TurretSubsystem() {
    // In intialization, find abosolute position and set the motor internal state
    double initialRotations = calculateAbsoluteRotations();
    if (initialRotations != -1) {
      double initialDegrees = initialRotations * 360.0;
      // Shift the "top half" of the circle to negative values
      if (initialDegrees > 180) {
        initialDegrees -= 360;
      }
      // turretSMC.setPosition(Degrees.of(initialDegrees));
      // System.out.print(initialDegrees);
    }
  }

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public void setAngleDirect(Angle angle) {
    turretSMC.setPosition(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return turret.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public Command sysId() {
    return turret.sysId(
        Volts.of(4.0), // maximumVoltage
        Volts.per(Second).of(0.5), // step
        Seconds.of(8.0) // duration
        );
  }

  public double getAngleDegrees() {
    return turret.getAngle().in(Degrees);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return turret.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }

  public void stop() {
    turretMotor.setControl(new VoltageOut(0));
  }

  public void setSpeed(double i) {
    turretMotor.setControl(new VoltageOut(i));
  }

  public double calculateAbsoluteRotations() {
    // Got Raw data from motors/encoders.
    double rawE1 = 0.0;
    double rawE2 = -0.088;
    double e1_val = (rawE1 - e1_offset) % 360;
    if (e1_val < 0) e1_val += 360;
    double e2_val = (rawE2 - e2_offset) % 360;
    if (e2_val < 0) e2_val += 360;
    for (int n1 = 0; n1 < e2_teeth; n1++) {
      double a1 = (n1 + (e1_val / 360)) * (e1_teeth / t_teeth);
      for (int n2 = 0; n2 < e1_teeth; n2++) {
        double a2 = (n2 + (e2_val / 360)) * (e2_teeth / t_teeth);
        if (Math.abs(a1 - a2) < 0.005) { // 0.005 is the tolerance for larger gears.
          if (a1 > 0.5) {
            return a1 - 1.0;
          }
          return a1; // turret position in rotations
        }
      }
    }
    return -1; // is no match is found
  }
}
