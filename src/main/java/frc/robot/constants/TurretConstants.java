package frc.robot.constants;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  // TODO: set according to wiring later
  public static final int ROTATOR_CANID = 5;
  public static final int SHOOTER_CANID = 6;
  public static final int HOOD_CANID    = 7;

  public static final int FEEDER_MOTOR_1_CANID = 8;
  public static final int FEEDER_MOTOR_2_CANID = 9;
  
  // TODO: Tune these
  // PID values for rotator and hood
  public static final double ROTATOR_P = 1.0;
  public static final double ROTATOR_I = 0.0;
  public static final double ROTATOR_D = 0.0;

  public static final double HOOD_P = 1.0;
  public static final double HOOD_I = 0.0;
  public static final double HOOD_D = 0.0;
  // no values for shooter as this is planned to be driven using a lookup table + power value

  // Rotation Limits for rotator and hood
  public static final Angle MIN_ROTATOR_ANGLE = Radians.of(- Math.PI / 2);
  public static final Angle MAX_ROTATOR_ANGLE = Radians.of(Math.PI / 2);

  public static final Angle MIN_HOOD_ANGLE = Radians.of(0);
  public static final Angle MAX_HOOD_ANGLE = Radians.of(Math.PI / 4);

  public static final double rotatorMotorRatio = 1;
  public static final double shooterMotorRatio = 1;
  public static final double hoodMotorRatio = 1;
}
