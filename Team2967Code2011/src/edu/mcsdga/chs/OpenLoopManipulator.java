// Columbus High IronWorks - FIRST Team 2967 - FRC 2011 Manipulator Class
// Class to control the forklift Jaguars, armArray Jaguar, and pneumatic claw functions.
// Class written by Erin King.

package edu.mcsdga.chs;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.can.*;

public class OpenLoopManipulator
{
    private boolean outerShouldBeUp, outerIsUp; // booleans for setting pneumatic outer lift position.
    private boolean liftThereYet, armThereYet;
    private Encoder innerLiftPosition;          // quad encoder for inner lift positioning.
    private AnalogChannel armPosition;          // potentiometer for arm positioning.

    private CANJaguar innerLift, arm;
    private static final double maxInnerLiftVoltage = 12;  // the arm motor should never get more than 12 volts.
    private static final double maxArmVoltage = 9;         // the arm motor should never get more than 9 volts.

    private double armLowerLimit = 0, armUpperLimit = 995, innerLiftUpperLimit = 16000;
    private static final double innerLiftAllowance = 100, armAllowance = 20;
    private static final double scoringPosition = 610.3;

    private DoubleSolenoid claw, outerLift;
    private static final double delaySeconds = 0.1;

    public OpenLoopManipulator()
    {
        // 3rd param - Inner lift encoder IS reversed.
        innerLiftPosition = new Encoder(10,11,true,Encoder.EncodingType.k2X);
        armPosition = new AnalogChannel(7);
        innerLiftPosition.start();

        // DIGITAL CHANNELS FOR PNEUMATIC SENSORS
        //12 is 4
        //13 is 5

        //solenoid to open and claw on channels 1 & 2 on solenoid breakout module.
        claw = new DoubleSolenoid(1,2);
        //solenoid to move the outer lift up and down on channels 3 & 4.
        outerLift = new DoubleSolenoid(3,4);

        try
        {
            // CHANNELS SET TEMPORARILY FOR PRACTICE BOT.
            innerLift = new CANJaguar(15, CANJaguar.ControlMode.kVoltage);
            arm = new CANJaguar(16, CANJaguar.ControlMode.kVoltage);
            innerLift.configNeutralMode(CANJaguar.NeutralMode.kBrake);
            arm.configNeutralMode(CANJaguar.NeutralMode.kBrake);
            innerLift.configMaxOutputVoltage(maxInnerLiftVoltage);
            arm.configMaxOutputVoltage(maxArmVoltage);
            // Calibrate sensors if a limit switch is triggered.
            if(!innerLift.getForwardLimitOK())
                innerLiftPosition.reset();
            if(!innerLift.getReverseLimitOK())
                innerLiftUpperLimit = innerLiftPosition.get();
            if(!arm.getReverseLimitOK())
                armLowerLimit = armPosition.getValue();
            if(!arm.getForwardLimitOK())
                armUpperLimit = armPosition.getValue();
        }
        catch (CANTimeoutException ex)
        {
            ex.printStackTrace();
            SmartDashboard.log("CANTimeoutException in OpenLoopManipulator constructor", "ERROR");
        }
    }
    // the outer lift will either be all the way down...
    public void moveOuterLiftDown()
    {
        if(outerIsUp)
        {
            outerLift.set(DoubleSolenoid.Value.kReverse);     // move the outer lift down.
            Timer.delay(delaySeconds);
            outerLift.set(DoubleSolenoid.Value.kOff);         // turn power off.
            outerIsUp = false;
        }
    }
    // ... or all the way up.
    public void moveOuterLiftUp()
    {
        if(!outerIsUp)
        {
            outerLift.set(DoubleSolenoid.Value.kForward);     // move the outer lift up.
            Timer.delay(delaySeconds);
            outerLift.set(DoubleSolenoid.Value.kOff);         // turn power off.
            outerIsUp = true;
        }
    }
    // move inner lift based on manual input.
    public void moveInnerLiftManual(double input)
    {
        try
        {
            // Joystick gives -1 to 1, convert to -12 to 12 volts.
            innerLift.setX(input * 12);

            // Calibrate values when a limit switch is triggered.
            if(!innerLift.getForwardLimitOK())
            {
                innerLiftPosition.reset();
                SmartDashboard.log("forward limit triggered", "inner limits");
            }
            if(!innerLift.getReverseLimitOK())
            {
                innerLiftUpperLimit = innerLiftPosition.get();
                SmartDashboard.log("reverse limit triggered", "inner limits");
            }
        }
        catch(CANTimeoutException ex)
        {
            ex.printStackTrace();
            SmartDashboard.log("CANTimeoutException in moveInnerLiftManual method", "ERROR");
        }
    }
    // move arm based on manual input.
    public void moveArmManual(double input)
    {
        try
        {
            // Joystick outputs -1 to 1: convert to -9 to 9 volts & invert for backwards motor.
            arm.setX(input * -9);   // -9 to 9 volts for moving arm up: tested.

            // Calibrate values when a limit switch is triggered.
            if(!arm.getReverseLimitOK())
            {
                armLowerLimit = armPosition.getValue();
                SmartDashboard.log("lower limit triggered","arm");
                SmartDashboard.log(armLowerLimit,"arm lower limit");
            }
            if(!arm.getForwardLimitOK())
            {
                armUpperLimit = armPosition.getValue();
                SmartDashboard.log("upper limit triggered","arm");
                SmartDashboard.log(armUpperLimit,"arm upper limit");
            }
        }
        catch(CANTimeoutException ex)
        {
            ex.printStackTrace();
            SmartDashboard.log("CANTimeoutException in moveArmManual method", "ERROR");
        }
    }
/*    // move both sections of the forklift using voltage based on limit switches.
    public void moveForkLiftManual(double input)
    {
        try
        {
            // Go half speed when going down.
            if (input<0)
            {
                // Once the inner lift triggers its lower limit, move the outer lift.
                if(innerLift.getForwardLimitOK())
                    innerLift.setX(input * -6);
                else
                    moveOuterLiftDown();
            }
            else if (input>0)
            {
                // Once the inner lift triggers its upper limit, move the outer lift.
                if(innerLift.getReverseLimitOK())
                    innerLift.setX(input * -12);
                else
                    moveOuterLiftUp();
            }
            else
                innerLift.setX(0);

            // Calibrate sensors if a limit switch is triggered.
            if(!innerLift.getForwardLimitOK())
                innerLiftPosition.reset();
            if(!innerLift.getReverseLimitOK())
                innerLiftUpperLimit = innerLiftPosition.get();
        }
        catch(CANTimeoutException ex)
        {
            ex.printStackTrace();
            SmartDashboard.log("CANTimeoutException in moveForkLift method", "ERROR");
        }
    }*/
    // the inner lift will use the encoder for position control through the cRIO.
    public boolean setInnerLift(double targetPosition)
    {
        SmartDashboard.log("setInnerLift method", "method status");
        boolean doneYet = false;
        // minVoltage and maxAdditionalVoltage must NOT add up to over 12.
        double minVoltage = 1, maxAdditionalVoltage = 5;
        double voltageToSend = 0;
        try
        {
            // if the target is below the current position...
            if(innerLiftPosition.get() > targetPosition + innerLiftAllowance)
            {
                // this is so the motor never gets higher than the maximum total voltage
                if(((innerLiftPosition.get()-targetPosition)/100)>1)
                    voltageToSend = minVoltage+maxAdditionalVoltage;
                // send a voltage according to the percentage remaining to the target,
                // and never give it lower than the minimum voltage.
                else
                    voltageToSend = minVoltage+(maxAdditionalVoltage*
                            ((innerLiftPosition.get()-targetPosition)/100));
                doneYet = false;
            }
            // if the target is above the current position...
            else if(innerLiftPosition.get() < targetPosition - innerLiftAllowance)
            {
                // this is so the motor never gets higher than the maximum total voltage
                if(((targetPosition-innerLiftPosition.get())/100)>1)
                    voltageToSend = -minVoltage-maxAdditionalVoltage;
                // send a voltage according to the percentage remaining to the target,
                // and never give it lower than the minimum voltage.
                else    //
                    voltageToSend = -minVoltage-(maxAdditionalVoltage*
                            ((targetPosition-innerLiftPosition.get())/100));
                doneYet = false;
            }
            else
                doneYet = true;
            // send voltage to the motor.
            // if the target position is all the way down...
            if(targetPosition <= 0)
            {
                // if the lower (forward) limit switch is not triggered...
                if(innerLift.getForwardLimitOK())
                {
                    //... go down full speed until it is.
                    voltageToSend = minVoltage+maxAdditionalVoltage;
                    doneYet = false;
                }
                else
                    doneYet = true;
            }
            // if the target position is all the way up...
            else if(targetPosition >= innerLiftUpperLimit || targetPosition >= 16000)
            {
                // if the upper (reverse) limit switch is not triggered...
                if(innerLift.getReverseLimitOK())
                {
                    // ...go up full speed until it is.
                    voltageToSend = -minVoltage-maxAdditionalVoltage;
                    doneYet = false;
                }
                else
                    doneYet = true;
            }
            innerLift.setX(voltageToSend);

            // Calibrate values when a limit switch is triggered.
            if(!innerLift.getForwardLimitOK())
                innerLiftPosition.reset();
            if(!innerLift.getReverseLimitOK())
                innerLiftUpperLimit = innerLiftPosition.get();
        }
        catch(CANTimeoutException ex)
        {
            ex.printStackTrace();
            SmartDashboard.log("CANTimeoutException in setInnerLift method", "ERROR");
        }
        return doneYet;
    }
    // the arm will use the potentiometer for position control through the cRIO.
    public boolean setArm(double targetPosition)
    {
        SmartDashboard.log("setArm method", "method status");
        boolean doneYet = false;
        // minVoltage and maxAdditionalVoltage must NOT add up to over 12.
        // for moving the arm, no more than 9 is recommended...
        //double minVoltage = 1, maxAdditionalVoltage = 3;
        double voltageToSend = 0;
        try
        {
            // if the target is below the current position...
            if(armPosition.getValue()>targetPosition+armAllowance)
            {
                SmartDashboard.log("going down", "arm");
                // THIS WORKS BETTER FOR THE ARM.
                voltageToSend = 5;
                doneYet = false;

                //if(((targetPosition-armPosition.getValue())/100)>1)
                //    voltageToSend = minVoltage+maxAdditionalVoltage;
                //else
                //    voltageToSend = minVoltage+(maxAdditionalVoltage*
                //        ((targetPosition - armPosition.getValue()) / 100));
            }
            // if the target is above the current position...
            else if(armPosition.getValue()<targetPosition-armAllowance)
            {
                SmartDashboard.log("going up", "arm");
                // THIS WORKS BETTER FOR THE ARM. NEEDS EXTRA VOLTAGE FOR GOING UP.
                voltageToSend = -6;
                doneYet = false;

                //if(((armPosition.getValue()-targetPosition)/100)>1)
                //    voltageToSend = -minVoltage-maxAdditionalVoltage;
                //else
                //    voltageToSend = -minVoltage-(maxAdditionalVoltage*
                //        ((armPosition.getValue()-targetPosition)/100));
            }
            else
                doneYet = true;
            // send voltage to the motors.
            arm.setX(voltageToSend);

            // Calibrate values when a limit switch is triggered.
            if(!arm.getReverseLimitOK())
            {
                armLowerLimit = armPosition.getValue();
                SmartDashboard.log("lower limit triggered","arm");
                SmartDashboard.log(armLowerLimit,"arm lower limit");
            }
            if(!arm.getForwardLimitOK())
            {
                armUpperLimit = armPosition.getValue();
                SmartDashboard.log("upper limit triggered","arm");
                SmartDashboard.log(armUpperLimit,"arm upper limit");
            }
        }
        catch(CANTimeoutException ex)
        {
            ex.printStackTrace();
            SmartDashboard.log("CANTimeoutException in setArm method", "ERROR");
        }
        return doneYet;
    }
    // move the lift autonomously to a certain position and move the arm accordingly.
    // NEEDS TO BE CALIBRATED
    public boolean moveToPreset(int preset)
    {
        SmartDashboard.log("moveToPreset method", "method status");
        double armTarget = armLowerLimit, innerTarget = 0;
        boolean doneYet;
        // arm positions - 0 = floor, 1 = tube-feeder slot height, 2 = straight up,
        // 3 = retracted, 4 = to place on the lower 2 pegs.

        // positions 1, 3, & 5 are low, middle, and high on shorter pole - arm all the way up.
        // positions 2, 4, & 6 are low, middle, and high on taller pole - arm all the way up.
        // position 7 is ground, for picking tubes up off the ground - arm all the way down.
        // position 8 is height of tube feeder slot - arm straight forward.
        // position 9 is retracted.
        switch (preset)
        {
            case 1: // low position on the shorter pole - arm is angled for bottom row.
                armTarget += scoringPosition;
                innerTarget = 0.0;//?
                //outerShouldBeUp = false;
                break;
            case 2: // low position on the taller pole - arm is angled for bottom row.
                armTarget += scoringPosition;
                innerTarget = 0.0;//?
                //outerShouldBeUp = false;
                break;
            case 3: // middle position on the shorter pole - arm is all the way up.
                armTarget += scoringPosition;
                innerTarget = 6624;
                //outerShouldBeUp = false;
                break;
            case 4: // middle position on the taller pole - arm is all the way up.
                armTarget += scoringPosition;
                innerTarget = 11617.5;
                //outerShouldBeUp = false;
                break;
            case 5: // high position on the shorter pole - arm is all the way up.
                armTarget += scoringPosition;
                innerTarget = 11967;
                //outerShouldBeUp = true;
                break;
            case 6: // high position on the taller pole - arm is all the way up.
                armTarget += scoringPosition;
                innerTarget = 16000;       //WILL GO UNTIL IT HITS LIMIT SWITCH
                //outerShouldBeUp = true;
                break;
            case 7: // ground - arm is all the way down.
                armTarget += 0;
                innerTarget = 0;
                //outerShouldBeUp = false;
                break;
            case 8: // height of the tube slot - arm is straight forward.
                //armTarget += 0;
                //innerTarget = 0.0;//?
                //outerShouldBeUp = false;
                break;
            case 9: // retracted up, for scoring in autonomous...
                armTarget += 995;
                innerTarget = 16000;    //WILL GO UNTIL IT HITS LIMIT SWITCH
                //outerShouldBeUp = false;
                break;
            case 10: // retracted.
                armTarget += 993;
                innerTarget = 0;
                //outerShouldBeUp = false;
                break;
            default:
                armTarget += scoringPosition;
                innerTarget = 0;
                //outerShouldBeUp = false;
                SmartDashboard.log("unrecognized position preset","error");
                break;
        }
        // if the inner lift is not at its target yet, send it there.
        if(!liftThereYet)
        {
            doneYet = false;
            SmartDashboard.log("setting inner lift","in move to preset");
            liftThereYet = setInnerLift(innerTarget);
        }
        // after lift has reached its target, stop that motor & send arm to its respective target.
        else if(!armThereYet)
        {
            doneYet = false;
            try{innerLift.setX(0);}
            catch(CANTimeoutException ex){ex.printStackTrace();}
            SmartDashboard.log("setting arm","in move to preset");
            armThereYet = setArm(armTarget);
        }
        // when both have reached their targets, stop the motors.
        else
        {
            doneYet = true;
            SmartDashboard.log("done","in move to preset");
            try{arm.setX(0);
                innerLift.setX(0);}
            catch(CANTimeoutException ex){ex.printStackTrace();}
        }
        // move the pneumatic outer lift up or down accordingly.
        if(outerShouldBeUp)
            moveOuterLiftUp();
        else
            moveOuterLiftDown();

        return doneYet;
    }
    // Grab a tube with the pneumatic claw.
    public void closeClaw()
    {
        claw.set(DoubleSolenoid.Value.kForward);     // Close the claw.
        Timer.delay(delaySeconds);
        claw.set(DoubleSolenoid.Value.kOff);         // Release power.
    }
    // Let go of a tube.
    public void openClaw()
    {
        claw.set(DoubleSolenoid.Value.kReverse);     // Open the claw.
        Timer.delay(delaySeconds);
        claw.set(DoubleSolenoid.Value.kOff);         // Release power.
    }
    // Revert to beginning-match configuration at the end of a match.
    public void endMatch()
    {
        //moveToPreset(10);                           // Retract lift and arm.
    }
    // get the current position of the inner lift.
    public double getInnerPosition()
    {
        return innerLiftPosition.get();
    }
    // get the current position of the arm.
    public double getArmPosition()
    {
        return armPosition.getValue();
    }
}