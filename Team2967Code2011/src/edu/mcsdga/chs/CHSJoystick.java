package edu.mcsdga.chs;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.SmartDashboard.*;

/**
 * CHSJoystick is an extension of the standard Joystick class written to handle
 * input from the Joysticks connected to the Columbus High Team 2967 IronDevils Driver Station.
 * This class adds functionality specifically for CHS's Driver Station. There is a single class
 * instance for each joystick and the mapping of ports to hardware buttons depends on the code
 * in the driver station. This class is written so that it can be seamlessly implemented wherever
 * the Joystick class is called simply by changing the declaration and instantiation.
 * This class corrects implementation quirks in the joystick, allows for output
 * modification such as maximums and dead zones, and toggling buttons.
 * @author Erin King, FIRST Team 2967. contact: king.erin.p@gmail.com
 * @version LogoMotion 2011
 */
public class CHSJoystick extends Joystick
{
    private boolean[] prevState;
    private boolean[] toggleState;
    private boolean wasDead = false;
    private final int numberOfAxes, throttleAxis;
    private static final int triggerButton = 1, topButton = 2;
    private double XNegDeadZone, XPosDeadZone, XMaxSpeedPercent;
    private double YNegDeadZone, YPosDeadZone, YMaxSpeedPercent;
    private double twistNegDeadZone, twistPosDeadZone, twistMaxSpeedPercent;
    private int myPort, XScale, YScale, twistScale;

    /**
     * Construct an instance of CHSJoystick.
     * The joystick index is the USB port on the drivers station.
     * CHSJoystick constructor allows user to configure dead zones, multipliers
     * (square/cube/etc. joystick axis inputs), and maximum outputs.
     * @param port Port on the driver station that the joystick is plugged into.
     * @param numAxes Number of axis types in the Joystick class enumeration.
     * @param numButtons Number of button types in the Joystick class enumeration.
     * @param XDZMin Minimum extreme of the x-axis dead zone, above which values will be returned as 0.
     * @param XDZMax Maximum extreme of the x-axis dead zone, below which values will be returned as 0.
     * @param XMult Multiplier for values from the X axis. 2 will square input, 3 will cube input, etc.
     * @param XMax Maximum (absolute value) output for X axis: should be a percentage (<1) or 1 for 100%.
     *          Inputs will be multiplied by this value before being returned. To invert the axis, make this value negative.
     * @param YDZMin Minimum extreme of the y-axis dead zone, above which values will be returned as 0.
     * @param YDZMax Maximum extreme of the y-axis dead zone, below which values will be returned as 0.
     * @param YMult Multiplier for values from the Y axis. 2 will square input, 3 will cube input, etc.
     * @param YMax Maximum (absolute value) output for Y axis: should be a percentage (<1) or 1 for 100%.
     *          Inputs will be multiplied by this value before being returned. To invert the axis, make this value negative.
     * @param twistDZMin Minimum extreme of the twist or z-axis dead zone, above which values will be returned as 0.
     * @param twistDZMax Maximum extreme of the tiwst or z-axis dead zone, below which values will be returned as 0.
     * @param twistMult Multiplier for values from the twist/Z axis. 2 will square input, 3 will cube input, etc.
     * @param twistMax Maximum (absolute value) output for twist/Z axis: should be a percentage (<1) or 1 for 100%.
     *          Inputs will be multiplied by this value before being returned. To invert the axis, make this value negative.
     */
    public CHSJoystick(int port, int numAxes, int numButtons, double XDZMin,
            double XDZMax, int XMult, double XMax, double YDZMin, double YDZMax, int YMult,
            double YMax, double twistDZMin, double twistDZMax,
            int twistMult, double twistMax)
    {
        // Tell it how many axes and buttons there are. Add 1 because 0th array index in parent class will be empty.
        super(port,(numAxes+1),(numButtons+1));
        super.setAxisChannel(AxisType.kX, 1);
        super.setAxisChannel(AxisType.kY, 2);
        super.setAxisChannel(AxisType.kZ, 3);           // Unused in this class.
        super.setAxisChannel(AxisType.kTwist, 3);       // Will not care if joystick has no twist axis.
        if(numAxes>3)
        {
            throttleAxis = 4;
            super.setAxisChannel(AxisType.kThrottle, 4);// Unused in this class.
        }
        else
            throttleAxis = 3;

        myPort = port;
        prevState = new boolean[numButtons+1];          // Add 1 because the 0th index will be empty.
        toggleState = new boolean[numButtons+1];        // Add 1 because the 0th index will be empty.
        numberOfAxes = numAxes;
        XNegDeadZone = XDZMin;
        XPosDeadZone = XDZMax;
        XMaxSpeedPercent = XMax;
        YNegDeadZone = YDZMin;
        YPosDeadZone = YDZMax;
        YMaxSpeedPercent = YMax;
        twistNegDeadZone = twistDZMin;
        twistPosDeadZone = twistDZMax;
        twistMaxSpeedPercent = twistMax;
        XScale = XMult;
        YScale = YMult;
        twistScale = twistMult;
    }

    /**
     * CHSJoystick three-parameter constructor. Sets number of axes and number of buttons,
     * and defaults to no dead zones, no limit on max outputs, and squaring inputs.
     * @param port The port on the driver station that the joystick is plugged into.
     */
    public CHSJoystick(int port, int numAxes, int numButtons)
    {
        // Default to no dead zones, no limit on max outputs, and squaring inputs.
        this(port, numAxes, numButtons, 0, 0, 2, 1, 0, 0, 2, 1, 0, 0, 2, 1);
    }

    /**
     * CHSJoystick default one-parameter constructor. Sets default dead zone, multiplier, and maximum output values.
     * Defaults to 4 axes, 12 buttons, no dead zones, no limit on max outputs, and squaring inputs.
     * @param port The port on the driver station that the joystick is plugged into.
     */
    public CHSJoystick(int port)
    {
        // Default to 5 axes, 12 buttons, no dead zones, no limit on max outputs, and squaring inputs.
        this(port, 5, 12, 0, 0, 2, 1, 0, 0, 2, 1, 0, 0, 2, 1);
    }

    /**
     * Get the X value of the current joystick.
     * This depends on the mapping of the joystick connected to the current port.
     * Method implements dead zone, multiplier, and max speed percent initialized in constructor.
     * @override getX() method in parent Joystick class.
     * @param hand Passed to super method - unused.
     * @return The X value of the joystick.
     */
    public double getX(Hand hand)
    {
        double X = super.getX(hand);
        SmartDashboard.log(X,"port "+myPort+" original X");
        // Configure Dead Zone from negDeadZone to posDeadZone on twist axis.
        if (((X*128)>XNegDeadZone)&&((X*128)<XPosDeadZone))
            X = 0.0;
        else
        {
            // Multiply inputs.
            for(int i = 0; i<XScale; i++)
                X *= Math.abs(X);
            // Multiply by max output.
            X*=XMaxSpeedPercent;
        }
        SmartDashboard.log(X,"port "+myPort+" X speed");
        return X;
    }

    /**
     * Get the Y value of the current joystick.
     * This depends on the mapping of the joystick connected to the current port.
     * Method implements dead zone, multiplier, and max speed percent initialized in constructor.
     * @override getY() method in parent Joystick class.
     * @param hand Passed to super method - unused.
     * @return The Y value of the joystick.
     */
    public double getY(Hand hand)
    {
        double Y = super.getY(hand);
        SmartDashboard.log(Y,"port "+myPort+" original Y");
        // Configure Dead Zone from negDeadZone to posDeadZone on twist axis.
        if (((Y*128)>YNegDeadZone)&&((Y*128)<YPosDeadZone))
            Y = 0.0;
        else
        {
            // Multiply inputs.
            for(int i = 0; i<YScale; i++)
                Y *= Math.abs(Y);
            // Multiply by max output, and invert because Y axis is inverted.
            Y*=YMaxSpeedPercent*-1;
        }
        SmartDashboard.log(Y,"port "+myPort+" Y speed");
        return Y;
    }

    /**
     * Get the twist value of the current joystick.
     * This depends on the mapping of the joystick connected to the current port.
     * Method implements dead zone, multiplier, and max speed percent initialized in constructor.
     * @override getTwist() method in parent Joystick class.
     * @return The twist/Z value of the joystick.
     */
    public double getTwist()
    {
        double twist = super.getTwist();
        SmartDashboard.log(twist,"port "+myPort+" original twist");
        // Configure Dead Zone from negDeadZone to posDeadZone on twist axis.
        if (((twist*128)>twistNegDeadZone)&&((twist*128)<twistPosDeadZone))
            twist = 0.0;
        else
        {
            // Multiply inputs.
            for(int i = 0; i<twistScale; i++)
                twist *= Math.abs(twist);
            // Multiply by max output.
            twist*=twistMaxSpeedPercent;
        }
        SmartDashboard.log(twist,"port "+myPort+" twist");
        return twist;
    }

    /**
     * Get the throttle value of the current joystick.
     * This depends on the mapping of the joystick connected to the current port.
     * @override getThrottle() method in parent Joystick class.
     * @return The throttle value of the joystick.
     */
    public double getThrottle()
    {
        // We use getRaw instead of getThrottle because the Joystick class calls
        // on a default axis, which may not work if the joystick used has
        // throttle on a different axis.
        double throttle = super.getRawAxis(throttleAxis);
        SmartDashboard.log(throttle,"port "+myPort+" original throttle");
        // Convert throttle from [1,-1] to [0,1].
        throttle = (throttle-1)/(-2);
        SmartDashboard.log(throttle,"port "+myPort+" throttle");
        return throttle;
    }

    /**
     * Get the Z value of the current joystick. If the joystick has over 3 axes, it returns
     * the same value as getTwist. Otherwise, it returns the same value as getThrottle.
     * This depends on the mapping of the joystick connected to the current port.
     * @override getZ() method in parent Joystick class.
     * @param hand Unused.
     * @return The twist/throttle/Z value of the joystick.
     */
    public double getZ(Hand hand)
    {
        if(numberOfAxes>3)
            return getTwist();
        return getThrottle();
    }

    /**
     * Get the value of the trigger on the current joystick.
     * This depends on the mapping of the joystick connected to the current port.
     * @override getTrigger() method in parent Joystick class.
     * @param hand Unused.
     * @return The trigger value of the joystick.
     */
    public boolean getTrigger(Hand hand)
    {
        return super.getRawButton(triggerButton);
    }

    /**
     * Get the value of the top button on the current joystick.
     * This depends on the mapping of the joystick connected to the current port.
     * @override getTop() method in parent Joystick class.
     * @param hand Unused.
     * @return The top button value of the joystick.
     */
    public boolean getTop(Hand hand)
    {
        return super.getRawButton(topButton);
    }

    /**
     * Returns true if all axes are within their dead zones, meaning all
     * joystick outputs are zero. Does not take throttle into account.
     * @return If the joystick is stopped.
     */
    public boolean isDead()
    {
        boolean dead = true;
        if(this.getX()!=0.0)
            dead = false;
        if(this.getY()!=0.0)
            dead = false;
        if(numberOfAxes>3 && this.getTwist()!=0.0)
            dead = false;
        if(dead && wasDead)
            return true;
        wasDead = dead;
        return false;
    }

    /**
     * Button toggle method. Press a button once to return true; press it again to return false.
     * Method prevents unintentional toggling by keeping track of the current button state.
     * @param button Which button is used to toggle the state.
     * @return The current toggle state of the button.
     */
    public boolean toggleButton(int button)
    {
        boolean state = super.getRawButton(button);
        if (state&&state!=prevState[button])
        {
            if (toggleState[button])
                toggleState[button] = false;
            else
                toggleState[button] = true;
        }
        prevState[button] = state;
        return toggleState[button];
    }

    /**
     * Reset toggle method makes the toggle state for the button false.
     * Sets previous state to the state before reset.
     * @param button Which button is used for the toggle.
     */
    public void resetToggle(int button)
    {
        prevState[button] = false;
        toggleState[button] = false;
    }

    /**
     * Set a dead zone for the X axis on the current joystick.
     * @param minimum Minimum extreme of the x-axis dead zone, above which values will be returned as 0.
     * @param maximum Maximum extreme of the x-axis dead zone, below which values will be returned as 0.
     */
    public void setXDeadZone(double minimum, double maximum)
    {
        XNegDeadZone = minimum;
        XPosDeadZone = maximum;
    }

    /**
     * Set maximum (absolute value) output for the X axis on the current joystick.
     * maxOutput should be a percentage (<1) or 1 for 100%. Inputs will be multiplied
     * by this value before being returned. To invert the axis, make this value negative.
     * @param maxOutput Maximum (absolute value) output of the x-axis.
     */
    public void setXMaxOutput(double maxOutput)
    {
        XMaxSpeedPercent = maxOutput;
    }

    /**
     * Set rate/curve for values from the X axis. 2 will square input, 3 will cube input, etc.
     * @param curve Scale/Rate for values from the X axis.
     */
    public void setXScale(int curve)
    {
        XScale = curve;
    }

    /**
     * Set a dead zone for the Y axis on the current joystick.
     * @param minimum Minimum extreme of the y-axis dead zone, above which values will be returned as 0.
     * @param maximum Maximum extreme of the y-axis dead zone, below which values will be returned as 0.
     */
    public void setYDeadZone(double minimum, double maximum)
    {
        YNegDeadZone = minimum;
        YPosDeadZone = maximum;
    }

    /**
     * Set maximum (absolute value) output for the Y axis on the current joystick.
     * maxOutput should be a percentage (<1) or 1 for 100%. Inputs will be multiplied
     * by this value before being returned. To invert the axis, make this value negative.
     * @param maxOutput Maximum (absolute value) output of the y-axis.
     */
    public void setYMaxOutput(double maxOutput)
    {
        YMaxSpeedPercent = maxOutput;
    }

    /**
     * Set rate/curve for values from the Y axis. 2 will square input, 3 will cube input, etc.
     * @param curve Scale/Rate for values from the Y axis.
     */
    public void setYScale(int curve)
    {
        YScale = curve;
    }

    /**
     * Set a dead zone for the twist axis on the current joystick.
     * @param minimum Minimum extreme of the twist axis dead zone, above which values will be returned as 0.
     * @param maximum Maximum extreme of the twist axis dead zone, below which values will be returned as 0.
     */
    public void setTwistDeadZone(double minimum, double maximum)
    {
        twistNegDeadZone = minimum;
        twistPosDeadZone = maximum;
    }

    /**
     * Set maximum (absolute value) output for the twist axis on the current joystick.
     * maxOutput should be a percentage (<1) or 1 for 100%. Inputs will be multiplied
     * by this value before being returned. To invert the axis, make this value negative.
     * @param maxOutput Maximum (absolute value) output of the twist axis.
     */
    public void setTwistMaxOutput(double maxOutput)
    {
        twistMaxSpeedPercent = maxOutput;
    }

    /**
     * Set rate/curve for values from the twist axis. 2 will square input, 3 will cube input, etc.
     * @param curve Scale/Rate for values from the twist axis.
     */
    public void setTwistScale(int curve)
    {
        twistScale = curve;
    }

    /**
     * Get rate/curve for values from the X axis. 2 will square input, 3 will cube input, etc.
     * @return Scale/Rate for values from the X axis.
     */
    public int getXScale()
    {
        return XScale;
    }

    /**
     * Get rate/curve for values from the Y axis. 2 will square input, 3 will cube input, etc.
     * @return Scale/Rate for values from the Y axis.
     */
    public int getYScale()
    {
        return YScale;
    }

    /**
     * Get rate/curve for values from the twist axis. 2 will square input, 3 will cube input, etc.
     * @return Scale/Rate for values from the twist axis.
     */
    public int getTwistScale()
    {
        return twistScale;
    }
}