package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class RunFlywheel extends Command {
  public FlywheelSubsystem flywheelSubsystem;

  public RunFlywheel(FlywheelSubsystem flywheelSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;
  }

  public void initialize() {
    // put things that need to be initialized here (such as a timer). No need to @Override.
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    flywheelSubsystem.setVelocity(AngularVelocity.ofRelativeUnits(180, DegreesPerSecond));
  }

  @Override
  public void end(boolean interrupted) {
    // this gets called when the input stops being given.
    flywheelSubsystem.setVelocity(AngularVelocity.ofRelativeUnits(0, DegreesPerSecond));
  }

  @Override
  public boolean isFinished() {
    // If true is returned, the command will stop being run. Can be used to check if a encoder is at
    // right place or limit switch is press (for example)
    return false;
  }
}
