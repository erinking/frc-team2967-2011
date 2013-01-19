/*----------------------------------------------------------------------------/
/ Copyright (c) FIRST 2008. All Rights Reserved.                             /
/ Open Source Software - may be modified and shared by FRC teams. The code   /
/ must be accompanied by the FIRST BSD license file in the root directory of /
/ the project.                                                               /
/----------------------------------------------------------------------------*/
// Columbus High IronWorks - FIRST Team 2967 - FRC 2011 Robot Code - Logomotion
// Code written by Erin King, Thomas Jones, and David Howard
// Programming Mentors - Paul King and David Rush

package edu.mcsdga.chs;

//import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO.EnhancedIOException;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

// The VM is configured to automatically run this class, and to call the
// functions corresponding to each mode, as described in the IterativeRobot
// documentation. If you change the name of this class or the package after
// creating this project, you must also update the manifest file in the resource
// directory.
public class Team2967Code2011 extends IterativeRobot
{
    // Declare variable for the robot drive system
    private RobotDrive driveTrain;          // robot will use CAN for drive motors

    // Declare variables for the two joysticks being used
    private CHSJoystick pilotStick;            // joystick 1 (Driver stick)
    private CHSJoystick copilotStick;          // joystick 2 (Manipulator stick)

    private CANJaguar frontLeft;
    private CANJaguar frontRight;
    private CANJaguar rearLeft;
    private CANJaguar rearRight;

    private Encoder LFenc, RFenc, LRenc, RRenc;              // encoders.

    private Compressor compressor;

    private OpenLoopManipulator liftArm;

    private AnalogTrigger photoSensorLeft, photoSensorMid, photoSensorRight;

    private DoubleSolenoid minibotDeploy;

    // STATE VARIABLES:
    private boolean isClosed = false, shouldBeClosed, shouldBeDeployed, isDeployed = false;
    private boolean fieldOriented = false, outerIsUp = false, outerShouldBeUp, wasEnabled;
    private boolean light1, light2, light3, followRight, forking = false, wasFollowing, following;
    private boolean movingToPreset = false, co3wasPressed, placingTube;
    private boolean donePositioningAutonomous, doneFollowingAutonomous;
    private boolean startedPlacingAutonomous, shouldBeClosedAutonomous, doneTiltingAutonomous;
    private boolean doneSecuring = false, doneClearing = false; // for the scoreTopFour() method

    private int autoPosition = 0, autoMode = 0, autoState = 0;
    private int teleopPosition = 0, teleopSwitchesOff, prevCopilotXscale;

    private double direction, throttle, leftSpeed, rightSpeed, X;
    private double scoringPosition = 610.3, startingLiftPosition, autonomousStartingPosition;
    // distance per pulse of a drive-wheel encoder, in inches.
    private static final double clickDistance = (Math.PI)/45;
    private static final double delaySeconds = 0.1;

    private String pilotMode = "ROBOT ORIENTED";      // vs. FIELD ORIENTED
    private String copilotMode = "LIFT/ARM CONTROL";    // vs. LIFT/ARM CONTROL
    private String autonomousState = "...";

    private Gyro gyro;                                       // gyro.

    //private AnalogChannel temperature;

    //private Accelerometer accel;                           // accelerometer.

    private AxisCamera camera;
    private Servo camX, camY;

    private DriverStationEnhancedIO enhancedIO;
    // Since the board was stupid and all the wires were tangled, we moved them so the
    // top row of switches is odd and the bottom row is even, and compensated for it in code.
    int [] switchBox = {0,1,3,5,7,9,11,13,15,2,4,6,8,10,12,14,16};

    SensorsManager sensorManager;

    // This function is run when the robot is first started up and should be
    // used for any initialization code.
    public void robotInit()
    {
        // DRIVER STATION INPUTS / OUTPUTS
        // Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
        // Pilot Stick is on port 1, with 4 axes and 12 buttons.
        // Dead zone from -15 to 10 on twist, square and cube inputs. No X & Y dead zones.
        pilotStick = new CHSJoystick(1,4,12,-5,5,2,1,-5,5,2,1,-15,10,3,0.7);
        // Copilot Stick is on port 2, with 3 axes and 11 buttons.
        // Dead zones from -10 to 10 on X & Y, square and quadruple inputs. No twist.
        // Negate X because the X axis is backwards on the Attack 3.
        // Negate Y because both the camera and the arm will be used airplane-style.
        copilotStick = new CHSJoystick(2,3,11,-10,10,2,-1,-10,10,4,-1,0,0,0,0);
        enhancedIO = DriverStation.getInstance().getEnhancedIO();
        SmartDashboard.init();

        // DIGITAL CHANNELS
        LFenc = new Encoder(1,2,true);      // LF encoder, digital ports 1 & 2, not goingBackwards.
        RFenc = new Encoder(3,4,false);     // RF encoder, digital ports 3 & 4, goingBackwards.
        LRenc = new Encoder(5,6,true);      // LR encoder, digital ports 5 & 6, not goingBackwards.
        RRenc = new Encoder(7,8,false);     // RR encoder, digital ports 7 & 8, goingBackwards.
        // Set distance per pulse for each encoder
        LFenc.setDistancePerPulse(clickDistance);
        RFenc.setDistancePerPulse(clickDistance);
        LRenc.setDistancePerPulse(clickDistance);
        RRenc.setDistancePerPulse(clickDistance);
        // Start the encoders.
        LFenc.start();
        LRenc.start();
        RFenc.start();
        RRenc.start();

        //accelerometer on I2C on digital sidecar, slot 1, with range from -4Gs to +4Gs
        //accel = new ADXL345_I2C(1,ADXL345_I2C.DataFormat_Range k4G);
        //[To access accelerations on 3 axes in Gs: accel.getAccelerations();]

        camera = AxisCamera.getInstance();
        camera.writeResolution(AxisCamera.ResolutionT.k640x360);
        camera.writeCompression(27);

        // SERVO CHANNELS
        camX = new Servo(5);
        camY = new Servo(6);

        // ANALOG CHANNELS
        gyro = new Gyro(1);                         //gyro on analog channel 1
        gyro.reset();

        photoSensorLeft=new AnalogTrigger(3);       // Channel 3 on Slot 1 on the cRIO
        photoSensorMid=new AnalogTrigger(4);        // Channel 5 on Slot 1 on the cRIO
        photoSensorRight=new AnalogTrigger(5);      // Channel 7 on Slot 1 on the cRIO
        // Value range to sense if photosensor is reading light (true) or dark (false)
        photoSensorMid.setLimitsVoltage(0.5,0.5);   // sets up per and lower value
        photoSensorLeft.setLimitsVoltage(0.5,0.5);  // limits for voltage readout,
        photoSensorRight.setLimitsVoltage(0.5,0.5); // for the Trigger to return true.

        //temperature = new AnalogChannel(6);         // Temperature Sensor on Analog Channel 6

        try
        {
        // CAN ID's
            // Define CAN Jaguar objects, set the drive Jags to be default voltage
            // controllers, & set the forklift Jags to be position controllers.
            frontLeft = new CANJaguar(11);
            frontRight = new CANJaguar(12);
            rearLeft = new CANJaguar(13);
            rearRight = new CANJaguar(14);

            frontLeft.configNeutralMode(CANJaguar.NeutralMode.kBrake);
            frontRight.configNeutralMode(CANJaguar.NeutralMode.kBrake);
            rearLeft.configNeutralMode(CANJaguar.NeutralMode.kBrake);
            rearRight.configNeutralMode(CANJaguar.NeutralMode.kBrake);

            liftArm = new OpenLoopManipulator();  // CANs 15, 16, and 17
        }
        catch (CANTimeoutException ex)
        {
            ex.printStackTrace();
        }
        compressor = new Compressor(9,1);         //1st param = digital channel for pressure switch
        compressor.start();                       //2nd = relay output for compressor; starts compressor.

        // SOLENOID CHANNELS
        minibotDeploy = new DoubleSolenoid(7,1,2);  //solenoid to deploy minibot.

        // Create a robot using standard right/left robot drive on the CAN bus
        driveTrain = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
        // Invert left side
        driveTrain.setInvertedMotor(RobotDrive.MotorType.kFrontRight, false);
        driveTrain.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
        driveTrain.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        driveTrain.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);

        sensorManager = SensorsManager.getInstance();
        sensorManager.start();
        sensorManager.waitUntilReady();
    }

/*    public void autonomousInit()
    {
        isClosed = false;
        donePositioningAutonomous = false;
        doneFollowingAutonomous = false;
        doneTiltingAutonomous = false;
        shouldBeClosedAutonomous = false;
        startedPlacingAutonomous = false;
        autoState = 1;
        liftArm.closeClaw();
        //sensorManager.startCountdown(0,15);
    }
    // This function is called periodically during autonomous
    public void autonomousPeriodic()
    {
        wasEnabled = true;
        // Use line follow sensors to move and then place ubertube.
        double scoreAutonomous = 660;

        // SWITCH 1 WILL GO STRAIGHT AND PLACE ON HIGH POLE, FOR EITHER THE LEFT OR RIGHT POSITION.
        // SWITCH 2 IS FOR THE MIDDLE POSITION TO GO LEFT AT THE FORK.
        // SWITCH 3 IS FOR THE MIDDLE POSITION TO GO RIGHT AT THE FORK - FROM THE ROBOT'S PERSPECTIVE!
        try
        {
            for (int i = 1; i < 4; i++)
              if (enhancedIO.getDigital(switchBox[i])==true)
                  autoMode = i;
            followRight = enhancedIO.getDigital(switchBox[3]);
        }
        catch (EnhancedIOException ex)
        {
            ex.printStackTrace();
        }
        // if we're on one of the outer two lines, go to the highest peg on the tall pole.
        //if (autoMode == 1)
            autoPosition = 9;
        // if we're on one of the inner two lines, on the Y, go to the highest peg on the short pole.
        //else if (autoMode == 2 || autoMode == 3)
        //    autoPosition = 5;

        switch(autoState)
        {
            case 1:
                if(!isClosed)
                {
                    liftArm.closeClaw();
                    isClosed = true;
                }
                shouldBeClosedAutonomous = true;
                autonomousState = "positioning mechanisms";
                //Go to the highest peg on the pole.
                if(!donePositioningAutonomous)
                    donePositioningAutonomous = liftArm.moveToPreset(autoPosition);
                else
                    autoState = 2;
                break;
            case 2:
                shouldBeClosedAutonomous = true;
                autonomousState = "following line.";
                // follow the line until you reach the end.
                if(!doneFollowingAutonomous)
                    doneFollowingAutonomous = lineFollow();
                else
                    autoState = 3;
                break;
            case 3:
                shouldBeClosedAutonomous = true;
                autonomousState = "tilting arm.";
                // tilt arm.
                if(!doneTiltingAutonomous)
                    doneTiltingAutonomous = liftArm.setArm(scoreAutonomous);
                else
                    autoState = 4;
                break;
            case 4:
                autonomousState = "placing ubertube.";
                // release the ubertube.
                /*if(!startedPlacingAutonomous)
                {
                    autonomousStartingPosition = liftArm.getInnerPosition();
                    startedPlacingAutonomous = true;
                }
                shouldBeClosedAutonomous = scoreTopFour(autonomousStartingPosition);*//*
                if(isClosed)
                {
                    liftArm.openClaw();
                    isClosed = false;
                }
                else
                    autoState = 5;
                shouldBeClosedAutonomous = false;
                break;
            case 5:
                shouldBeClosedAutonomous = false;
                liftArm.setArm(scoringPosition);
                break;
        }

        if(!isClosed&&shouldBeClosedAutonomous)
        {
            liftArm.closeClaw();
            isClosed = true;
        }
        else if(isClosed&&!shouldBeClosedAutonomous)
        {
            liftArm.openClaw();
            isClosed = false;
        }

        // Debug readouts on LCD:
        SmartDashboard.log(autonomousState,"autonomous state");
    }
    public void teleopInit()
    {
        autoState = 0;
    }*/
    // This function is called periodically during operator control
    public void teleopPeriodic()
    {
        wasEnabled = true;
        // **********PILOT CONTROLS**********
        // Pilot Joystick controls drivetrain, encoders, line followers, & gyro.

        // Press button 11 to switch from field-oriented to robot-oriented.
        if (pilotStick.getRawButton(11)&&fieldOriented)
            fieldOriented = false;
        else if (pilotStick.getRawButton(12)&&!fieldOriented)
            fieldOriented = true;
        if (fieldOriented)          // Use Gyro output for field-oriented control.2mnh/
        {
            direction=gyro.getAngle();
            pilotMode = "ROBOT ORIENTED";
        }
        else                        // Use 0 as direction for robot-oriented control.
        {
            direction=0;
            pilotMode = "FIELD ORIENTED";
        }

        //throttle = pilotStick.getThrottle();
        throttle = 1;

        // Joystick input controls robot based on cartesian mecanum drive.
        if (!pilotStick.isDead())
            driveTrain.mecanumDrive_Cartesian(
                (-pilotStick.getX()*throttle),
                (pilotStick.getY()*throttle),
                (-pilotStick.getTwist()*throttle), direction);

        // Pull the trigger to reset gyroscope.
        if (pilotStick.getTrigger())
            gyro.reset();

        // If switch 5 is on, follow the right line if/when you get to the fork.
        try{followRight = enhancedIO.getDigital(switchBox[4]);}
        catch(EnhancedIOException ex){ex.printStackTrace();}
        // Press button 3 once to follow a line.
        // To stop before it has reached the end, press button 3 again or move the joystick.
        following = pilotStick.toggleButton(3);
        if (!pilotStick.isDead())
            following = false;
        if (following)
            lineFollow();
        // ^COMMENT THIS OUT FOR COMPETITION^

        // Press button 4 to reset encoder distances.
        if (pilotStick.getRawButton(4))
        {
            LFenc.reset();                      //reset left front encoder
            RFenc.reset();                      //reset right front encoder
            LRenc.reset();                      //reset left rear encoder
            RRenc.reset();                      //reset right rear encoder
        }

        // Press button 7 to deploy or retract the minibot actuator.
        shouldBeDeployed = pilotStick.toggleButton(7);
        if(!isDeployed&&shouldBeDeployed)
        {
            minibotDeploy.set(DoubleSolenoid.Value.kForward); // Deploy.
            Timer.delay(delaySeconds);
            minibotDeploy.set(DoubleSolenoid.Value.kOff);                // Release power.
            isDeployed = true;
        }
        else if(isDeployed&&!shouldBeDeployed)
        {
            minibotDeploy.set(DoubleSolenoid.Value.kReverse); // Retract.
            Timer.delay(delaySeconds);
            minibotDeploy.set(DoubleSolenoid.Value.kOff);     // Release power.
            isDeployed = false;
        }

        // **********CO-PILOT CONTROLS**********
        // Co-Pilot Joystick controls camera servos, forklift motors, & pneumatic actuators.

        // Hold down button 7 to switch to FORWARDS camera control instead of grabber control.
        if (copilotStick.getRawButton(7))
        {
            // Control the camera-orientation servo motors.
            copilotMode = "FORWARD CAMERA CONTROL";
            prevCopilotXscale = copilotStick.getXScale();          // retain the previous scale.
            copilotStick.setXScale(1);                             // set the scale to 1.
            camX.set(((copilotStick.getX())+1)/4);     // X control of half of the servo range.
            camY.set(((copilotStick.getY())+1)/2);     // Y control stays the same.
        }
        // Hold down button 6 to switch to REVERSE camera control.
        else if(copilotStick.getRawButton(6))
        {
            // Control the camera-orientation servo motors.
            copilotMode = "REVERSE CAMERA CONTROL";
            prevCopilotXscale = copilotStick.getXScale();          // retain the previous scale.
            copilotStick.setXScale(1);                             // set the scale to 1.
            camX.set(((copilotStick.getX())+3)/4);     // X control of the other half of the servo range.
            camY.set(((copilotStick.getY())+1)/2);     // Y control stays the same.
        }
        // Grabber control.
        else
        {
            // Move X-axis to move arm, or Y-axis to move forklift.
            copilotMode = "LIFT/ARM CONTROL";
            copilotStick.setXScale(prevCopilotXscale);             // set the scale to the previous scale.
            if (!copilotStick.isDead())
            {
                liftArm.moveInnerLiftManual(copilotStick.getX());
                liftArm.moveArmManual(copilotStick.getY());
            }
            // Press button 4 to move the outer lift up or down.
            outerShouldBeUp = copilotStick.toggleButton(4);
            if(outerShouldBeUp&&!outerIsUp)
            {
                liftArm.moveOuterLiftUp();
                outerIsUp = true;
            }
            else if (outerIsUp&&!outerShouldBeUp)
            {
                liftArm.moveOuterLiftDown();
                outerIsUp = false;
            }
        }

        // Pull trigger to grab a tube with the pneumatic claw. It will wait until
        // tube sensor is triggered to close. Press again to let go manually.
        shouldBeClosed = copilotStick.toggleButton(1);

        // Press button 3 to automatically place the tube.
        // Override if the claw should be open or closed if used...
        placingTube = copilotStick.getRawButton(3);
        if(placingTube)
        {
            if(!co3wasPressed)
                startingLiftPosition = liftArm.getInnerPosition();
            shouldBeClosed = scoreTopFour(startingLiftPosition);
            //goDistance(-15.0); // move back 15 inches.
        }
        else
        {
            doneSecuring = false;
            doneClearing = false;
        }
        co3wasPressed = placingTube;

        // send commands to the claw...
        if(!isClosed&&shouldBeClosed)
        {
            liftArm.closeClaw();
            isClosed = true;
        }
        else if(isClosed&&!shouldBeClosed)
        {
            liftArm.openClaw();
            isClosed = false;
        }

        // Turn on the switch to decide which position to move to automatically.
        try
        {
            teleopSwitchesOff = 0;
            for (int i = 1; i <= 8; i++)
            {
              if (enhancedIO.getDigital(switchBox[i+8]))
                  teleopPosition = i;
              else
                  teleopSwitchesOff++;
            }
            if(teleopSwitchesOff==8)
                teleopPosition = 0;
        }
        catch (EnhancedIOException ex)
        {
            ex.printStackTrace();
        }
        // Press button 2 to automatically go to a certain height based on switches 9 to 16.
        if(copilotStick.getRawButton(2))
        {
            movingToPreset = true;
            liftArm.moveToPreset(teleopPosition);
            // If going to ground or slot & the claw is closed, open the claw.
            //if((teleopPosition==7||teleopPosition==8)&&isClosed)
            //{
            //    liftArm.openClaw();
            //    isClosed = false;
            //}
        }
        else if(movingToPreset)
        {
            liftArm.moveInnerLiftManual(0);
            liftArm.moveArmManual(0);
            movingToPreset = false;
        }

        // Debug readouts on SmartDashboard:
        SmartDashboard.log("main teleOp method", "method status");
        SmartDashboard.log(pilotMode, "pilot control mode");
        SmartDashboard.log(copilotMode, "copilot control mode");
        SmartDashboard.log(camX.get(), "Camera X");
        SmartDashboard.log(camY.get(), "Camera Y");
        SmartDashboard.log(teleopPosition, "teleOp height target");
        //SmartDashboard.log(getAverageDriveTrainDistance(),"Avg. Distance");
        //SmartDashboard.log(LFenc.getDistance(), "left front");
        //SmartDashboard.log(RFenc.getDistance(), "right front");
        //SmartDashboard.log(LRenc.getDistance(), "left rear");
        //SmartDashboard.log(RRenc.getDistance(), "right rear");
        SmartDashboard.log(photoSensorLeft.getTriggerState(),"left photosensor");
        SmartDashboard.log(photoSensorMid.getTriggerState(),"middle photosensor");
        SmartDashboard.log(photoSensorRight.getTriggerState(),"right photosensor");
        //SmartDashboard.log(camera.getCompression(),"camera compression level");
        SmartDashboard.log(camera.freshImage(),"Is camera image fresh?");
        SmartDashboard.log(gyro.getAngle(), "gyro angle");
        SmartDashboard.log(compressor.getPressureSwitchValue(),"pressure switch");
        SmartDashboard.log(compressor.enabled(),"compressor enabled?");
        SmartDashboard.log(isDeployed,"minibot deployed?");
        SmartDashboard.log(liftArm.getInnerPosition(),"inner position in main");
        SmartDashboard.log(liftArm.getArmPosition(),"arm position in main");
    }

    // Line follower method.
    public boolean lineFollow()
    {
        boolean doneYet;
        int state = 0;
        final double minorAdjust = .2;
        final double majorAdjust = .5;
        final double shift = .3;
        String followString = "error";

        light1=photoSensorLeft.getTriggerState();
        light2=photoSensorMid.getTriggerState();
        light3=photoSensorRight.getTriggerState();

        // the left sees the line
        if (light1)
            state+=1;
        // the middle sees the line
        if (light2)
            state+=2;
        // the right sees the line
        if (light3)
            state+=4;

        switch (state)
        {
            case 0:         //no sensors are triggered - no line - continue forward.
                if (wasFollowing)   // Was following a line - stop.
                {
                    followString = "stopping";
                    following = false;
                    forking = false;
                }
                else                // Not following yet - look for the line.
                {
                    rightSpeed = .3;
                    leftSpeed = .3;
                    followString = "looking for line";
                }
                break;
            case 1:         //left only is triggered - correct left.
                if (wasFollowing)   // Already following a line - correct left.
                {
                    rightSpeed = majorAdjust;
                    leftSpeed = -majorAdjust;
                    X = -shift;
                    followString = "correcting left";
                }
                else                // Not following yet - orient towards the line.
                {
                    leftSpeed = majorAdjust;
                    rightSpeed = -majorAdjust;
                    followString = "orienting right";
                }
                break;
            case 2:         //middle only is triggered - continue forward.
                leftSpeed = .5;
                rightSpeed = .5;
                X = 0.0;
                followString = "moving forward";
                break;
            case 3:         //left and middle are triggered - correct mildly left.
                rightSpeed = minorAdjust;
                leftSpeed = -minorAdjust;
                X = -shift;
                followString = "correcting mildly left";
                break;
            case 4:         //right only is triggered.
                if (wasFollowing)   // Already following a line - correct right.
                {
                    leftSpeed = majorAdjust;
                    rightSpeed = -majorAdjust;
                    X = shift;
                    followString = "correcting right";
                }
                else                // Not following yet - orient towards the line.
                {
                    rightSpeed = majorAdjust;
                    leftSpeed = -majorAdjust;
                    followString = "orienting left";
                }
                break;
            case 5:         //right & left - hit fork - choose lane based on switch & go mecanum.
                //  if you're in autonomous mode 2 or switch 8 is on in teleop:
                if (!followRight)
                {
                    X = -shift;
                    followString = "following left line";
                }
                //  if you're in autonomous mode 3 or switch 8 is off in teleop:
                else if(followRight)
                {
                    X = shift;
                    followString = "following right line";
                }
                forking = true;
                break;
            case 6:         //right and middle are triggered - correct mildly right.
                leftSpeed = minorAdjust;
                rightSpeed = -minorAdjust;
                X = shift;
                followString = "correcting mildly right";
                break;
            case 7:         //all sensors are triggered - hit end of line marker - stop moving.
                followString = "stopping";
                following = false;
                forking = false;
                break;
        }
        leftSpeed*=-1;  //?
        rightSpeed*=-1; //?
        X*=-1;          //?
        if (following&&!forking)
        {
            driveTrain.setLeftRightMotorOutputs(leftSpeed, (-1 * rightSpeed));
            doneYet = false;
        }
        else if(following&&forking)
        {
            driveTrain.mecanumDrive_Cartesian(X, 0.2, 0, 0);
            doneYet = false;
        }
        else
        {
            followString = "done following";
            doneYet = true;
            if (autoState>0)    // If in autonomous, go on to next state.
                autoState++;
        }
        SmartDashboard.log(followString,"Line Follower");
        if (state > 0)
            wasFollowing = true;
        else
            wasFollowing = false;

        return doneYet;
    }

    // opens claw and gets out of the way - returns false beacuse claw is not closed.
    // if placing on the bottom row, angle and lift simultaneously to release.
    // else, drops lift down a bit.
    // RETURNS WHETHER OR NOT THE CLAW SHOULD BE CLOSED
    public boolean scoreTopFour(double startingPosition)
    {
        SmartDashboard.log("scoreTopFour method", "method status");
        // Number of counts to move the lift or rotations to move the arm to clear the peg.
        double secureTubeDown = 700, clearPegDown = 1000;
        boolean shouldBeClosedAuto = false;

        if(!doneSecuring)
        {
            shouldBeClosedAuto = true;
            liftArm.setArm(scoringPosition);
            doneSecuring = liftArm.setInnerLift(startingPosition-secureTubeDown);
        }
        else if(doneSecuring && !doneClearing)
        {
            shouldBeClosedAuto = false;
            doneClearing = liftArm.setInnerLift(startingPosition-clearPegDown);
        }
        else
            shouldBeClosedAuto = false;
        return shouldBeClosedAuto;
    }

    // Method to get the average encoder distances on the drivetrain wheels.
//    public double getAverageDriveTrainDistance()
//    {
//        return ((LFenc.getDistance()+RFenc.getDistance()+LRenc.getDistance()+RRenc.getDistance())/4);
//    }

    // Attempt at a method to use the encoder inputs to make the robot go straight forward
    // or backward a certain distance. Method also resets encoder distances after use...
/*    public void goDistance(double distance) // distance is in inches.
    {
        int goingBackwards;
        if (distance < 0)
            goingBackwards = -1;
        else
            goingBackwards = 1;
        distance *= goingBackwards;
        LFenc.reset();                      //reset left front encoder
        RFenc.reset();                      //reset right front encoder
        LRenc.reset();                      //reset left rear encoder
        RRenc.reset();                      //reset right rear encoder
        while((Math.abs((LFenc.getDistance()+RFenc.getDistance()+LRenc.getDistance()+RRenc.getDistance())/4))<distance)
        {
            //if ((Math.abs((LFenc.getDistance()+RFenc.getDistance()+LRenc.getDistance()+RRenc.getDistance())/4))<(distance/2))
            //    driveTrain.mecanumDrive_Cartesian(0,1.0*goingBackwards,0,0);
            //else if ((Math.abs((LFenc.getDistance()+RFenc.getDistance()+LRenc.getDistance()+RRenc.getDistance())/4))<(distance*2/3))
            //    driveTrain.mecanumDrive_Cartesian(0,0.75*goingBackwards,0,0);
            //else if ((Math.abs((LFenc.getDistance()+RFenc.getDistance()+LRenc.getDistance()+RRenc.getDistance())/4))<(distance*3/4))
                driveTrain.mecanumDrive_Cartesian(0,0.5*goingBackwards,0,0);
            //else
            //    driveTrain.mecanumDrive_Cartesian(0,0.25*goingBackwards,0,0);
        }
    }*/

    // Called when the robot is disabled.
    public void disabledPeriodic()
    {
        if (wasEnabled)
            endMatch();
    }

    // Method to revert all pneumatics and expanded parts back to their match beginning state.
    public void endMatch()
    {
        //minibotDeploy.set(DoubleSolenoid.Value.kReverse);
        //Timer.delay(delaySeconds);
        //minibotDeploy.set(DoubleSolenoid.Value.kOff);
        fieldOriented = false;
        following = false;
        liftArm.endMatch();
        wasEnabled=false;
    }
}