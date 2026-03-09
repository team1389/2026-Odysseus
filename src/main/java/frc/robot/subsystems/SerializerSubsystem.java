package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
// import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

// import yams.motorcontrollers.remote.TalonFXSWrapper;

public class SerializerSubsystem extends SubsystemBase {

  private final TalonFX indexerMotor = new TalonFX(RobotMap.IndexerCanID);

  private final TalonFX kickerTopMotor = new TalonFX(RobotMap.KickerTopCanID);
  /*
  private final SmartMotorControllerConfig kickerTopMotorConfig =
      new SmartMotorControllerConfig(this)
          // Configure Motor and Mechanism properties
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false);

  @SuppressWarnings("unused")

  private final SmartMotorController KickerTopSMC =
      new TalonFXWrapper(kickerTopMotor, DCMotor.getKrakenX44Foc(1), kickerTopMotorConfig);
  */
  private final TalonFX kickerBottomMotor = new TalonFX(RobotMap.KickerBottomCanID);

  /*
    private final SmartMotorControllerConfig kickerBottomMotorConfig =
        new SmartMotorControllerConfig(this)
            // Configure Motor and Mechanism properties
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(true);

    @SuppressWarnings("unused")
    private final SmartMotorController KickerBottomSMC =
        new TalonFXWrapper(kickerBottomMotor, DCMotor.getKrakenX44Foc(1), kickerBottomMotorConfig);
  */
  public SerializerSubsystem() {
    // Check motor allignment
    kickerTopMotor.setControl(new Follower(RobotMap.IndexerCanID, MotorAlignmentValue.Aligned));
    kickerBottomMotor.setControl(new Follower(RobotMap.IndexerCanID, MotorAlignmentValue.Aligned));
  }

  public void setSpeed(double dutyCycleSpeed) {
    indexerMotor.setControl(new DutyCycleOut(dutyCycleSpeed));
  }

  public void SetVoltage(double voltage) {
    indexerMotor.setVoltage(voltage);
  }

  public void stop() {
    indexerMotor.setControl(new VoltageOut(0));
  }

  @Override
  public void periodic() {}
}
