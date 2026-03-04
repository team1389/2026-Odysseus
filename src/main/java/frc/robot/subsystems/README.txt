Implement all subsystems by extending SubsystemBase
Simple subsystem code example below

package frc.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase{
    private TalonFX IntakeMotor = new TalonFX(RobotMap.IntakeCanID);
    public IntakeSubsystem() {
        IntakeMotor = new TalonFX(RobotMap.IntakePort);
        
    }

    public void setSpeed(double intakeSpeed){
        IntakeMotor.setControl(new VoltageOut(intakeSpeed));
    }
    public void stop(){
        IntakeMotor.setControl(new VoltageOut(0));
    }
}


