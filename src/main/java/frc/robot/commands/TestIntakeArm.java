package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TestIntakeArm extends Command {
  public IntakeSubsystem intakeSubsystem;
  public double targetRPM;

  public TestIntakeArm(IntakeSubsystem intakeSubsystem, double targetRPM) {
    this.intakeSubsystem = intakeSubsystem;
    this.targetRPM = targetRPM;
  }

  public void initialize() {
    // put things that need to be initialized here (such as a timer). No need to @Override.
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    intakeSubsystem.setArmVoltage(targetRPM);
  }

  @Override
  public void end(boolean interrupted) {
    // this gets called when the input stops being given.
    intakeSubsystem.stopArm();
  }

  @Override
  public boolean isFinished() {
    // If true is returned, the command will stop being run. Can be used to check if a encoder is at
    // right place or limit switch is press (for example)
    return false;
  }
}
