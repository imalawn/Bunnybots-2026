package frc.robot.util.io;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A version of {@link CommandGenericHID} with specific trigger factories for a Guitar Hero
 * controller.
 *
 * @see CommandGenericHID
 */
public class GuitarHeroController extends CommandGenericHID {
  /** Represents a digital button on a GuitarHeroController. */
  public enum Button {
    GREEN_FRET(1),
    RED_FRET(2),
    YELLOW_FRET(4),
    BLUE_FRET(3),
    ORANGE_FRET(5);

    /** Button value. */
    public final int value;

    Button(int value) {
      this.value = value;
    }
  }

  /** Represents an axis on a GuitarHeroController */
  public enum Axis {
    // strum bar is represented as a pov
    WHAMMY_BAR(4),
    TILT(5);

    /** Axis value. */
    public final int value;

    Axis(int value) {
      this.value = value;
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public GuitarHeroController(int port) {
    super(port);
  }

  /**
   * Constructs a Trigger instance around the green fret button's digital signal. Both the lower and
   * upper frets share the same signal.
   *
   * @return a Trigger instance representing the green fret button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger green() {
    return button(Button.GREEN_FRET.value);
  }

  /**
   * Constructs a Trigger instance around the red fret button's digital signal. Both the lower and
   * upper frets share the same signal.
   *
   * @return a Trigger instance representing the red fret button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger red() {
    return button(Button.RED_FRET.value);
  }

  /**
   * Constructs a Trigger instance around the yellow fret button's digital signal. Both the lower
   * and upper frets share the same signal.
   *
   * @return a Trigger instance representing the yellow fret button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger yellow() {
    return button(Button.YELLOW_FRET.value);
  }

  /**
   * Constructs a Trigger instance around the blue fret button's digital signal. Both the lower and
   * upper frets share the same signal.
   *
   * @return a Trigger instance representing the blue fret button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger blue() {
    return button(Button.BLUE_FRET.value);
  }

  /**
   * Constructs a Trigger instance around the orange fret button's digital signal. Both the lower
   * and upper frets share the same signal.
   *
   * @return a Trigger instance representing the orange fret button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger orange() {
    return button(Button.ORANGE_FRET.value);
  }

  /**
   * Constructs a Trigger instance around the axis value of the whammy bar. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [-1, 1] where -1 is the unpressed state of the axis.
   * @return a Trigger instance that is true when the whammy bar's axis exceeds the provided
   *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
   *     button loop}.
   */
  public Trigger whammyBar(double threshold) {
    return axisGreaterThan(Axis.WHAMMY_BAR.value, threshold);
  }

  /**
   * Constructs a Trigger instance around the axis value of the whammy bar. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @return a Trigger instance that is true when the whammy bar's axis exceeds 0.5, attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger whammyBar() {
    return whammyBar(0.5);
  }

  /**
   * Constructs a Trigger instance around the axis value of the guitar's tilt. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [-1, 1] where -1 is the unpressed state of the axis.
   * @return a Trigger instance that is true when the tilt's axis exceeds the provided threshold,
   *     attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button
   *     loop}.
   */
  public Trigger tilt(double threshold) {
    return axisGreaterThan(Axis.TILT.value, threshold);
  }

  /**
   * Constructs a Trigger instance around the axis value of the guitar's tilt. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @return a Trigger instance that is true when the tilt's axis exceeds 0.5, attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger tilt() {
    return tilt(0.5);
  }

  /**
   * Get the whammy bar axis value of the controller. Although this is represented as a floating
   * point number, the whammy bar only has two states (-1 and 0.98).
   *
   * @return The axis value.
   */
  public double getWhammyBarAxis() {
    return getRawAxis(Axis.WHAMMY_BAR.value);
  }

  /**
   * Get the tilt axis value of the controller. This is bound by [0.19, 1].
   *
   * @return The axis value.
   */
  public double getTiltAxis() {
    return getRawAxis(Axis.TILT.value);
  }
}
