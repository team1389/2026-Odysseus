package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends Command {
  public IntakeSubsystem intakeSubsystem;
  public double targetVoltage;

  public IntakeCmd(IntakeSubsystem intakeSubsystem, double targetVoltage) {
    this.intakeSubsystem = intakeSubsystem;
    this.targetVoltage = targetVoltage;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    intakeSubsystem.setRollerVoltage(targetVoltage);
  }

  @Override
  public void end(boolean interrupted) {
    // this gets called when the input stops being given.
    intakeSubsystem.stopRoller();
  }

  @Override
  public boolean isFinished() {
    // If true is returned, the command will stop being run. Can be used to check if a encoder is at
    // right place or limit switch is press (for example)
    return false;
  }
}
