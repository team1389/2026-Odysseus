package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCmd extends Command {
  public TurretSubsystem turretSubsystem;
  public double targetVoltage;

  public TurretCmd(TurretSubsystem turretSubsystem, double targetVoltage) {
    this.turretSubsystem = turretSubsystem;
    this.targetVoltage = targetVoltage;
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    turretSubsystem.setVoltage(-targetVoltage);
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
