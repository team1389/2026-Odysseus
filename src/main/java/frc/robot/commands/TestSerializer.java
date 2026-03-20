package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerializerSubsystem;

public class TestSerializer extends Command {
  public SerializerSubsystem serializerSubsystem;
  public double targetDutyCycle;

  public TestSerializer(SerializerSubsystem serializerSubsystem, double targetDutyCycle) {
    this.serializerSubsystem = serializerSubsystem;
    this.targetDutyCycle = targetDutyCycle;
    addRequirements(serializerSubsystem);
  }

  public void initialize() {
    // put things that need to be initialized here (such as a timer). No need to @Override.
  }

  @Override
  public void execute() {
    // This gets called when the command does.
    serializerSubsystem.setSpeed(targetDutyCycle);
  }

  @Override
  public void end(boolean interrupted) {
    // this gets called when the input stops being given.
    serializerSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    // If true is returned, the command will stop being run. Can be used to check if a encoder is at
    // right place or limit switch is press (for example)
    return false;
  }
}
