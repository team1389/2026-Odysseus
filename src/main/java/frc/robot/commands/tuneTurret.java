package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class tuneTurret extends Command {
  public TurretSubsystem turretSubsystem;
  public double targetAngle;

  public tuneTurret(TurretSubsystem turretSubsystem, double targetAngle) {
    this.turretSubsystem = turretSubsystem;
    this.targetAngle = targetAngle;
  }

  public void initialize() {
    // put things that need to be initialized here (such as a timer). No need to @Override.
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    turretSubsystem.setAngle(Angle.ofBaseUnits(targetAngle, Degrees));
  }

  @Override
  public void end(boolean interrupted) {
    // this gets called when the input stops being given.
    turretSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // If true is returned, the command will stop being run. Can be used to check if a encoder is at
    // right place or limit switch is press (for example)
    return false;
  }
}
