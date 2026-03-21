package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
// import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

// import yams.motorcontrollers.remote.TalonFXSWrapper;

public class SerializerSubsystem extends SubsystemBase {

  private final TalonFX indexerMotor = new TalonFX(RobotMap.IndexerCanID);

  private final TalonFX kickerTopMotor = new TalonFX(RobotMap.KickerTopCanID);

  private final TalonFX kickerBottomMotor = new TalonFX(RobotMap.KickerBottomCanID);

  // --- Mechanism2d ---
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1),
          DCMotor.getKrakenX60Foc(1));
  private final Mechanism2d mech = new Mechanism2d(2, 2);
  private final MechanismRoot2d root = mech.getRoot("SerializerRoot", 1, 1);

  private final MechanismLigament2d roller =
      root.append(
          new MechanismLigament2d(
              "Indexer Roller",
              0.6, // length
              0, // angle
              6, // thickness
              new Color8Bit(Color.kOrange)));

  public SerializerSubsystem() {
    // Check motor allignment
    kickerTopMotor.setControl(new Follower(RobotMap.IndexerCanID, MotorAlignmentValue.Aligned));
    kickerBottomMotor.setControl(new Follower(RobotMap.IndexerCanID, MotorAlignmentValue.Aligned));
    if (Robot.isSimulation()) {
      SmartDashboard.putData("Serializer Mech", mech);
    }
  }

  public void setSpeed(double dutyCycleSpeed) {
    indexerMotor.setControl(new DutyCycleOut(dutyCycleSpeed));
  }

  public void stop() {
    indexerMotor.setControl(new VoltageOut(0));
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // 1. Pass the real motor voltage to the simulation
    // TalonFX uses Volts for more accuracy than Duty Cycle
    motorSim.setInput(indexerMotor.getSimState().getMotorVoltage());

    // 2. Advance the physics world by the loop period (usually 0.02s)
    motorSim.update(0.020);

    // 3. Update the hardware sim state so the TalonFX "thinks" it's moving
    var simState = indexerMotor.getSimState();
    simState.setRawRotorPosition(motorSim.getAngularPositionRotations());
    simState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60.0);

    // 4. Update your visualizer
    roller.setAngle(Units.rotationsToDegrees(motorSim.getAngularPositionRotations()));
  }
}
