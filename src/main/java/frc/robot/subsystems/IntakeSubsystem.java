package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor = new TalonFX(RobotMap.IntakeCanID);
  private final TalonFX intakeArmMotor = new TalonFX(RobotMap.IntakeArmCanID);

  // Roller Simulation
  private static final double intakeMotorSimGearRatio = 3.0;

  @SuppressWarnings("unused")
  private final FlywheelSim intakeWheelSim =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(0.01, 0.001),
          DCMotor.getKrakenX60Foc(1),
          intakeMotorSimGearRatio);

  private final DCMotorSim intakeMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), 0.001, intakeMotorSimGearRatio),
          DCMotor.getKrakenX60Foc(1));

  // Arm Configuration (YAMS)
  private final SmartMotorControllerConfig intakeArmMotorConfig =
      new SmartMotorControllerConfig(this)
          .withGearing(new MechanismGearing(GearBox.fromTeeth(75, 1)))
          .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
          .withTelemetry("IntakeArmMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withClosedLoopController(
              new ProfiledPIDController(
                  1.0, 0.0, 0.0, new Constraints(Math.toRadians(0), Math.toRadians(0))))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);

  private final SmartMotorController intakeArmSMC =
      new TalonFXWrapper(intakeArmMotor, DCMotor.getKrakenX60(1), intakeArmMotorConfig);

  private final Arm intakeArm =
      new Arm(
          new ArmConfig(intakeArmSMC)
              .withMass(Pounds.of(6.3857643))
              .withStartingPosition(Degrees.of(90))
              .withTelemetry("IntakeArmMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
              .withMOI(0.081916)
              .withLength(Inches.of(22.938))
              .withHardLimit(Degrees.of(-20), Degrees.of(100)));

  // Arm Simulation
  private final Mechanism2d mech = new Mechanism2d(60, 60); // A 60x60 inch canvas
  private final MechanismRoot2d armRoot = mech.getRoot("ArmRoot", 30, 30); // Center of canvas
  private final MechanismLigament2d armVisual =
      armRoot.append(
          new MechanismLigament2d("IntakeArm", 22.9, 90)); // Name, length, starting angle
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          75.0, // Gearing
          0.0078, // MOI
          0.58, // Length (meters)
          Math.toRadians(-20), // Min Angle
          Math.toRadians(100), // Max Angle
          true, // Simulate Gravity
          Math.toRadians(90) // Starting Angle
          );
  private final MechanismLigament2d rollerVisual =
      armVisual.append(new MechanismLigament2d("Rollers", 4, 0, 6, new Color8Bit(0, 255, 0)));

  public IntakeSubsystem() {
    if (Robot.isSimulation()) {
      SmartDashboard.putData("Intake Mech", mech);
    }
  }

  // Private Roller Control

  public void setRollerVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  public void setRollerVoltage(Supplier<Double> volts) {
    intakeMotor.setVoltage(volts.get());
  }

  public void stopRoller() {
    intakeMotor.setControl(new NeutralOut());
  }

  // Arm Commands

  public Command setAngle(Angle angle) {
    return intakeArm.setAngle(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return intakeArm.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return intakeArm.getAngle();
  }

  public void setArmVoltage(double volts) {
    intakeArmMotor.setVoltage(volts);
  }

  public void setArmVoltage(Supplier<Double> volts) {
    intakeArmMotor.setVoltage(volts.get());
  }

  public void stopArm() {
    intakeArmMotor.setVoltage(0);
  }

  // Combined Intake Commands
  public Command intake(Angle angle) {
    return run(() -> setRollerVoltage(1.0)).alongWith(intakeArm.setAngle(angle));
  }

  public Command intake(Supplier<Angle> angleSupplier) {
    return run(() -> setRollerVoltage(1.0)).alongWith(intakeArm.setAngle(angleSupplier));
  }

  public Command outtake(Angle angle) {
    return run(() -> setRollerVoltage(-1.0)).alongWith(intakeArm.setAngle(angle));
  }

  public Command outtake(Supplier<Angle> angleSupplier) {
    return run(() -> setRollerVoltage(-1.0)).alongWith(intakeArm.setAngle(angleSupplier));
  }

  public Command retract(Angle retractAngle) {
    return runOnce(this::stopRoller).andThen(intakeArm.setAngle(retractAngle));
  }

  // public Command runRollers(double targetRPM) {
  //   return run(() -> setRollerVoltage(targetRPM));
  // }

  // SysId

  public Command sysId() {
    return intakeArm.sysId(Volts.of(4.0), Volts.per(Second).of(0.5), Seconds.of(8.0));
  }

  @Override
  public void periodic() {
    intakeArm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // 1. Get the voltage the Talon is applying
    var simState = intakeArmMotor.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    double armMotorVoltage = simState.getMotorVoltageMeasure().in(Volts);

    // 2. Update the Physics Model
    armSim.setInput(armMotorVoltage);
    armSim.update(0.020); // 20ms loop

    // 3. Update the Hardware Sensors
    double sensorPos = armSim.getAngleRads() * (75.0 / (2 * Math.PI));
    simState.setRawRotorPosition(sensorPos);

    // Set velocity so PID/Feedforward works correctly
    double sensorVel = armSim.getVelocityRadPerSec() * (75.0 / (2 * Math.PI));
    simState.setRotorVelocity(sensorVel);
    armVisual.setAngle(Math.toDegrees(armSim.getAngleRads()));

    var talonFXSim = intakeMotor.getSimState();
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    intakeMotorSim.setInputVoltage(motorVoltage.in(Volts));
    intakeMotorSim.update(0.020);

    talonFXSim.setRawRotorPosition(
        intakeMotorSim.getAngularPosition().times(intakeMotorSimGearRatio));

    talonFXSim.setRotorVelocity(intakeMotorSim.getAngularVelocity().times(intakeMotorSimGearRatio));
    rollerVisual.setAngle(intakeMotor.getPosition().getValueAsDouble() * 360.0);
  }
}
