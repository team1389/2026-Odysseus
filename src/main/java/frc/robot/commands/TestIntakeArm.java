package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.Supplier;

public class TestIntakeArm extends Command {
  public IntakeSubsystem intakeSubsystem;
  public Supplier<Double> volts;

  public TestIntakeArm(IntakeSubsystem intakeSubsystem, Supplier<Double> volts) {
    this.intakeSubsystem = intakeSubsystem;
    this.volts = volts;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    intakeSubsystem.setArmVoltage(() -> volts.get() * 16);
  }

  @Override
  public void end(boolean interrupted) {
    // this gets called when the input stops being given.
    intakeSubsystem.stopArm();
  }
}
