package frc.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.*;
import java.util.function.Supplier;

public class joystickHoodCommand extends Command {
  HoodSubsystem hoodSubsystem;
  private Supplier<Double> power;

  public joystickHoodCommand(HoodSubsystem hoodSubsystem, Supplier<Double> power) {
    this.hoodSubsystem = hoodSubsystem;
    this.power = power;
    addRequirements(hoodSubsystem);
  }

  @Override
  public void execute() {
    hoodSubsystem.setHoodVoltage(MathUtil.clamp(power.get(), -10, 10));
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.stop();
  }
}
