package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
// import yams.motorcontrollers.local.SparkWrapper;
// import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {

  private final Distance flywheelDiameter = Inches.of(4);
  // Changed the motor type to TalonFX, added second motor
  private final TalonFX flywheelMotor1 = new TalonFX(RobotMap.FlywheelCanID);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              0.08, 0, 0.17, RPM.of(500000), RotationsPerSecondPerSecond.of(2500))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor =
      new TalonFXWrapper(flywheelMotor1, DCMotor.getKrakenX60(1), motorConfig);

  // Added correct values for diameter and mass
  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(motor)
          .withDiameter(Inches.of(4))
          .withMass(Pounds.of(5))
          .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
          .withSoftLimit(RPM.of(-5000), RPM.of(5000))
          .withSpeedometerSimulation(RPM.of(7500));

  // Its only one flywheel because a flywheel that has twice the mass of the actual flywheel on the
  // robot would work the same as both flywheels do.
  private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

  public FlywheelSubsystem() {}

  public void setSpeed(double volts) {
    flywheelMotor1.setVoltage(volts);
  }

  public AngularVelocity getVelocity() {
    return flywheel.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return flywheel.setSpeed(speed);
  }

  public Command setDutyCycle(double dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return flywheel.setSpeed(speed);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  public Command sysId() {
    return flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
  }

  public double getSpeedRPM() {
    return flywheel.getSpeed().in(RPM);
  }

  @Override
  public void periodic() {
    flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    flywheel.simIterate();
  }

  public Command setRPM(LinearVelocity speed) {
    return flywheel.setSpeed(
        RotationsPerSecond.of(
            speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }

  public void setRPMDirect(LinearVelocity speed) {
    motor.setVelocity(
        RotationsPerSecond.of(
            speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }

  public void setRPM(AngularVelocity speed) {
    motor.setVelocity(speed);
  }
}
