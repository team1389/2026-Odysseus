package frc.robot;

/** define Hardware Ports and CAN id's in here */
public class RobotMap {
  // Drivetrain Motor Ports

  // Drivetrain CAN id's

  // Other CAN id's
  // Turret CAN id's
  public static final int TurretCanID = 25;

  // Max Speeds
  public static final double MaxTurretSpeed = 1;
  public static final int FlywheelCanID = 28;

  // Henry's Hood
  public static final int HoodCanID = 27;
  public static final double HoodIntegralCorr =
      0.00016541; // How aggressivly hood fixes position error
  public static final int HoodMaxAmp = 40; // Max motor current allowed (avoid brown out)
  public static final double HoodRampRatePID =
      .25; // In seconds time to ramp from 0 to full output in PID mode
  public static final double HoodRampRateMan =
      .25; // In seconds time to ramp from 0 to full output in manual mode
  public static final int HoodGearRatio = 4;
  public static final double HoodStaticVolts =
      0.27937; // Volts needed to overcome static friction and make hood move
  public static final double HoodVelVolts = 0.089836; // Volts needed to maintain const velocity
  public static final double HoodAccVolts = 0.014557; // Voltage needed per unit of acceleration
  public static final boolean HoodMoterInvert =
      false; // True if the hood moter is inverted relative to the angle rotation
  public static final int HoodMaxVel = 5000; // Max allowed rotational velocity in hood
  public static final int HoodMaxAcc = 2500; // Max allowed velocity change in hood

  // Intake Motor ports
  public static final int IntakeMotorPort = 15;
  public static final int IntakeArmMotorPort = 16;
  // Other CAN id's
  public static final int IntakeCanID = 21;
  public static final int IntakeArmCanID = 20;
  public static final double IntakeArmAngle = 0;

  // Other CAN id's

  public static final int IndexerCanID = 22;
  public static final int KickerTopCanID = 24;
  public static final int KickerBottomCanID = 23;

  // Speed constants

  public static final double SerializerSpeed = 0.75;
}
