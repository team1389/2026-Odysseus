package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {
  private final TalonFX hoodMotor = new TalonFX(RobotMap.HoodCanID);
  private final SmartMotorControllerConfig hoodMotorConfig =
      new SmartMotorControllerConfig(this)
          .withGearing(
              new MechanismGearing(
                  GearBox.fromReductionStages(144.9, 1))) // gear ratio after reduction
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(RobotMap.HoodMaxAmp))
          .withMotorInverted(RobotMap.HoodMoterInvert)
          .withClosedLoopRampRate(Seconds.of(RobotMap.HoodRampRatePID))
          .withClosedLoopController(
              new ProfiledPIDController(
                  120.0, 0.0, 0.0, new Constraints(Math.toRadians(10), Math.toRadians(30))))
          .withOpenLoopRampRate(Seconds.of(RobotMap.HoodRampRateMan))
          .withFeedforward(new SimpleMotorFeedforward(0.6, 8.0, 0.07))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController hoodSMC =
      new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX44(1), hoodMotorConfig);

  private final ArmConfig hoodConfig =
      new ArmConfig(hoodSMC)
          .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
          .withSoftLimits(Degrees.of(2), Degrees.of(100))
          .withHardLimit(
              Degrees.of(0), Degrees.of(68)); // The Hood can be modeled as an arm since it has a
  // gravitational force acted upon based on the angle its in

  private final Arm hood = new Arm(hoodConfig);

  public HoodSubsystem() {}

  public Command setAngle(Angle angle) {
    return hood.setAngle(angle);
  }

  public void setAngleDirect(Supplier<Angle> angle) {
    hoodSMC.setPosition(angle.get());
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return hood.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return hood.getAngle();
  }

  public Command sysId() {
    return hood.sysId(
        Volts.of(4.0), // maximumVoltage
        Volts.per(Second).of(0.5), // step
        Seconds.of(8.0) // duration
        );
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return hood.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  public void setHoodVoltage(double volts) {
    hoodMotor.setVoltage(volts);
  }

  public void stop() {
    hoodMotor.setVoltage(0);
  }

  public double getAngleDegrees() {
    return hood.getAngle().in(Degrees);
  }


  @Override
  public void periodic() {
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }
}
