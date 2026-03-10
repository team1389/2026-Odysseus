package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class tuneHood extends Command {
  public HoodSubsystem hoodSubsystem;
  public double targetAngle;

  public tuneHood(hoodSubsystem hoodSubsystem, double targetAngle) {
    this.hoodSubsystem = hoodSubsystem;
    this.targetAngle = targetAngle;
  }

  public void initialize() {
    // put things that need to be initialized here (such as a timer). No need to @Override.
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    hoodSubsystem.setAngle(targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    // this gets called when the input stops being given.
    hoodSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // If true is returned, the command will stop being run. Can be used to check if a encoder is at
    // right place or limit switch is press (for example)
    return false;
  }
}
