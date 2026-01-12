package frc.robot.constants;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  // TODO: set according to wiring later
  public static final Integer ROTATOR_CANID = 1;
  public static final Integer SHOOTER_CANID = 2;
  public static final Integer HOOD_CANID    = 3;
  
  // TODO: Tune these
  // PID values for rotator and hood
  public static final Double ROTATOR_P = 1.0;
  public static final Double ROTATOR_I = 0.0;
  public static final Double ROTATOR_D = 0.0;

  public static final Double HOOD_P = 1.0;
  public static final Double HOOD_I = 0.0;
  public static final Double HOOD_D = 0.0;
  // no values for shooter as this is planned to be driven using a lookup table + power value

  // Rotation Limits for rotator and hood
  public static final Angle MIN_ROTATOR_ANGLE = Radians.of(- Math.PI / 2);
  public static final Angle MAX_ROTATOR_ANGLE = Radians.of(Math.PI / 2);

  public static final Angle MIN_HOOD_ANGLE = Radians.of(0);
  public static final Angle MAX_HOOD_ANGLE = Radians.of(Math.PI / 4);
}
