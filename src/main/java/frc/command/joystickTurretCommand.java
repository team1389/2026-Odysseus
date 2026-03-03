package frc.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.*;
import java.util.function.Supplier;

public class joystickTurretCommand extends Command {
  TurretSubsystem turretSubsystem;
  private Supplier<Double> power;

  public joystickTurretCommand(TurretSubsystem turretSubsystem, Supplier<Double> power) {
    this.turretSubsystem = turretSubsystem;
    this.power = power;
    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    turretSubsystem.setSpeed(MathUtil.clamp(power.get(), -10, 10));
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }
}
