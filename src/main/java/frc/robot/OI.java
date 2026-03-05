package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.subsystems.VisionSubsystem;


public class OI {
    //Define controller ports | DO NOT TOUCH |
    final CommandXboxController manipController = new CommandXboxController(1);
    final CommandXboxController driveController = new CommandXboxController(0);
    public static VisionSubsystem visionSubsystem = new VisionSubsystem();

    //Define all subsystems using "IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();"
    //IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();



    //YAGSL Swerve Data Stream

    /*
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    private final TargetingSystem targetingSystem = new TargetingSystem();
                                                                                                                                                      
  
    //Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.


    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> driveController.getLeftY() * -1,
                                                                    () -> driveController.getLeftX() * -1) 
                                                                    //Raw axis of rightTriggerAxis is 3 for some reason
                                                                .withControllerRotationAxis(() -> driveController.getRightTriggerAxis() * -1)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);
    
    
    //Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
    
    
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driveController::getRightX,
                                                                                                driveController::getRightY)
                                                            .headingWhile(true);
    
    //Clone's the angular velocity input stream and converts it to a robotRelative input stream.
    
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                .allianceRelativeControl(false);
    */
  
    //Creates Bindings for controllers
    public OI() {
        configureBindings();
        
    }


    private void configureBindings() {
        /*
        Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngle);

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        //EDIT YOUR COMMANDS HERE_______________________________________________________________________________________________________________________________
        driveController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
         */
        //PUT YOUR COMMANDS in here! Default commands go first.  
        //manipController.a().whileTrue(new Intake(IntakeSubsystem));
        //manipController.a().whileTrue(new RunVision(visionSubsystem));
    }

    public Command getAutonomousCommand() {
        // AUTOS not PATHS in path planner should be called here
        return new PathPlannerAuto("AutoName");
    }


}