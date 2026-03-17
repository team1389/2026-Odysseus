package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.Supplier;

public class TestIntakeArm extends Command {
  public IntakeSubsystem intakeSubsystem;
  public Supplier<Double> angle;

  public TestIntakeArm(IntakeSubsystem intakeSubsystem, Supplier<Double> angle) {
    this.intakeSubsystem = intakeSubsystem;
    this.angle = angle;
  }

  public void initialize() {
    // put things that need to be initialized here (such as a timer). No need to @Override.
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    intakeSubsystem.setArmVoltage(() -> angle.get() * 16);
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
