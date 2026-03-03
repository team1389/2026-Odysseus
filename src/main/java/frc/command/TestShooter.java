package frc.command;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.FlywheelSubsystem;

public class TestShooter extends Command {
  public FlywheelSubsystem flywheelSubsystem;
  public double targetRPM;

  public TestShooter(FlywheelSubsystem flywheelSubsystem, double targetRPM) {
    this.flywheelSubsystem = flywheelSubsystem;
    this.targetRPM = targetRPM;
  }

  public void initialize() {
    // put things that need to be initialized here (such as a timer). No need to @Override.
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    flywheelSubsystem.setRPMDirect(LinearVelocity.ofBaseUnits(targetRPM, MetersPerSecond));
  }

  @Override
  public void end(boolean interrupted) {
    // this gets called when the input stops being given.
    flywheelSubsystem.setRPMDirect(LinearVelocity.ofBaseUnits(0, InchesPerSecond));
  }

  @Override
  public boolean isFinished() {
    // If true is returned, the command will stop being run. Can be used to check if a encoder is at
    // right place or limit switch is press (for example)
    return false;
  }
}
