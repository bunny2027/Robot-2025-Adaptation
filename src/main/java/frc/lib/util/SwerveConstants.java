package frc.robot.lib.util;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final double canCoderMagnetOffset;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param canCoderMagnetOffset
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int canCoderID, double canCoderMagnetOffset) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = canCoderID;
    this.canCoderMagnetOffset = canCoderMagnetOffset;
  }
}

