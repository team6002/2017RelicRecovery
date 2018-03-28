package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PIDController;

//import for Rev Robotics IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
//end of import for Rev Robotics IMU


public class RobotHardware extends LinearOpMode
{


  ElevatorThread elevatorThread = new ElevatorThread();
  IntakeThread intakeThread = new IntakeThread();

    //start of IMU class.
  // The IMU sensor object
  BNO055IMU imu;

  // State used for updating telemetry
  public Orientation angles;
  public Acceleration gravity;
  public  Orientation lastAngles = new Orientation();
  public double                  globalAngle, power = .30,correction;
  

//end of IMU class

    private DcMotor motorLeft, motorRight;

    private DcMotor intakeLeft, intakeRight;

    private DcMotor elevatorLift;
    private DcMotor armRotate;
    private Servo armLeft,armRight;
    private Servo jewelServo;

    private DcMotor relicArm;
    private Servo relicWrist, relicClaw;

    double JEWEL_DOWN = 0.3;//0.2 == arm on the ground
    double JEWEL_DOWN2 = JEWEL_DOWN + .03;//0.2 == arm on the ground
    double JEWEL_UP = 0.87;  //0.8 == arm straight up in the air

    private ColorSensor colorSensor;

    private DigitalChannel glyphSwitch;
    private DigitalChannel switchLeftFront;
    private DigitalChannel switchRightFront;
    private DigitalChannel alignmentLeft;
    private DigitalChannel alignmentRight;
    private DigitalChannel switchElevatorHome;

    final double INCH_2_TICKS_FACTOR = ((28*30)/(4*3.14))*(48/48.5); // (PPR * motor reduction) / (Wheel Diamemter*PI)

    double INTAKE_POWER = 0.4;
    double FRONT_SWITCH_INTAKE_POWER = .5;
    double LEFT_ALIGNMENT_INTAKE_POWER =  INTAKE_POWER*1.5;
    double RIGHT_ALIGNMENT_INTAKE_POWER = INTAKE_POWER*1.5;



    double left_intake_power = 0;
    double right_intake_power = 0;

    public enum IntakeStates {
        STOP
        ,FORWARD
        ,REVERSE
    }

    IntakeStates intakeState = IntakeStates.STOP;

    public enum Intake_WantedStates {
      STOP
      , FORWARD
      , REVERSE
    }

    Intake_WantedStates intake_WantedState = Intake_WantedStates.STOP;

    public enum ClawStates {
      MIN // not a state, used to hold min value
      , CLOSE // close to hold a glyph
      , OPEN_SLIGHTLY
      , OPEN_REGULAR
      , OPEN_WIDE
      , MAX // not a state, used to hold max value
    }

    double[] clawPositionTicks = {0.0,0.45,0.52,0.61,.98,1.0}; // right claw values, (left = 1-right)
    ClawStates clawState = ClawStates.OPEN_REGULAR;

    double ARM_ROTATE_POWER = 1.0;
    double ARM_ROTATE_POWER_FAST = 1.0;

    public enum ArmStates {
      MIN // not a state, used to hold min value
      , FRONT
      , MID
      , TILT_BACK
      , BACK
      , TILT_FORWARD
      , MAX // not a state, used to hold max value
    }
    int[] armPositionTicks = {0,0,-800,-1450,-1580,-1800,-1800};
    ArmStates armState = ArmStates.MIN;

    double RELIC_ARM_POWER = 1.0;

    public enum RelicArmStates {
      MIN
      , HOME // arm retracted and claw closed
      , EXTENDED // arm extended ready to capture
      , CAPTURED // close claw and move arm back to home position
      , TIPPED // tip the wrist back to clear the 12in wall
      , PLACED // place relic on the ground
      , RETRACTED // arm retracted, wrist in up position
      , MAX
    }

    int [] relicArmPositionTicks = {0,0,2290, 2290};

    RelicArmStates relicArmState = RelicArmStates.HOME;


    double ELEVATOR_POWER = 1.0;
    public enum ElevatorStates{
      MIN // not a state - used to hold min position
      , LOW
      , RESTACK
      , HALF
      , STACK
      , HIGH
      , MAX // not a state - used to hold max position
    };

    int[] elevatorPositionTicks = {0,0,633,760,910,1010,1010};
    ElevatorStates elevatorState = ElevatorStates.LOW;



    public enum WantedStates {
      STAY // do nothing
      , HOME
      , STACK
      , SINGLE
      , RESTACK
      , RAISE_LOWER
      , TILT_ARM_FORWARD
      , TILT_ARM_BACK
      , RELIC_NEXT_STEP
      , RELIC_HOME
    }

    WantedStates wantedState  = WantedStates.STAY;


    //all the states the robot can be in


    public enum SystemState{
      HOME //no glyphs, arms open in bot.
      , HIGH //no glyphs, elevator in high position inside bot
      , LOW //no glyphs, elevator in low position inside bot (claw opened or closed)
      , GLYPHS_1 //holding 1 glyphs inside robot
      , GLYPHS_2 //holding 2 glyphs inside robot
      , GLYPHS_HIGH //holding glyphs high behind bot
      , GLYPHS_LOW //holding glyphs low behing bot
      , TILT_ARM // tilt arm back to clear glyph
    }


    SystemState mSystemState = SystemState.HOME;
    //--------------------------------------------------------------------------
    //
    // PushBotHardware
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public RobotHardware()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotManual::PushBotHardware

    //--------------------------------------------------------------------------
    //
    // init
    //
    //--------
    // Performs any actions that are necessary when the OpMode is enabled.
    //
    // The system calls this member once when the OpMode is enabled.
    //--------
    @Override public void runOpMode() throws InterruptedException
    {
        //
        // Use the hardwareMap to associate class members to hardware ports.
        //
        // Note that the names of the devices (i.e. arguments to the get method)
        // must match the names specified in the configuration file created by
        // the FTC Robot Controller (Settings-->Configure Robot).
        //

        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        //Connect the intake motors
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        //Connect the elevator and arm motor
        elevatorLift = hardwareMap.get(DcMotor.class, "elevatorLift");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        //Connect the claw servos.
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        jewelServo = hardwareMap.get(Servo.class, "jewelServo");

        relicArm = hardwareMap.get(DcMotor.class, "relicArm");
        relicWrist = hardwareMap.get(Servo.class, "relicWrist");
        relicClaw = hardwareMap.get(Servo.class, "relicClaw");


        //Connect the intake switches
        glyphSwitch = hardwareMap.get(DigitalChannel.class, "glyphSwitch");
        alignmentLeft = hardwareMap.get(DigitalChannel.class, "alignmentLeft");
        alignmentRight = hardwareMap.get(DigitalChannel.class, "alignmentRight");
        switchLeftFront = hardwareMap.get(DigitalChannel.class, "switchLeftFront");
        switchRightFront = hardwareMap.get (DigitalChannel.class, "switchRightFront");
        switchElevatorHome = hardwareMap.get (DigitalChannel.class, "elevatorHome");
        colorSensor = hardwareMap.get (ColorSensor.class, "colorSensor");


        //
        // Connect the drive wheel motors.
        //
        // The direction of the right motor is reversed, so joystick inputs can
        // be more generically applied.
        //

        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorLift.setDirection(DcMotor.Direction.REVERSE);
        elevatorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLift.setTargetPosition(elevatorPositionTicks[ElevatorStates.LOW.ordinal()]);
        elevatorLift.setPower(ELEVATOR_POWER);

        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relicArm.setTargetPosition(relicArmPositionTicks[RelicArmStates.HOME.ordinal()]);
        relicArm.setPower(RELIC_ARM_POWER);
        relicWrist.setPosition(0);
        relicClaw.setPosition(0);


        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotate.setTargetPosition(armPositionTicks[ArmStates.FRONT.ordinal()]);
        armRotate.setPower(ARM_ROTATE_POWER);

        glyphSwitch.setMode(DigitalChannel.Mode.INPUT);
        switchElevatorHome.setMode(DigitalChannel.Mode.INPUT);
        jewelServo.setPosition(JEWEL_UP);
        
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode               = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled      = false;
        imuParameters.loggingTag          = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
        while (!imu.isGyroCalibrated()) {
          sleep(50);
        }

        composeTelemetry();

        elevatorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevatorThread.start();
        intakeThread.start();
        
        

    }
    


    //--------------------------------------------------------------------------
    //
    // loop
    //
    //--------
    // Performs any actions that are necessary while the OpMode is running.
    //
    // The system calls this member repeatedly while the OpMode is running.
    //--------

    //--------------------------------------------------------------------------
    //
    // stop
    //
    //--------
    // Performs any actions that are necessary when the OpMode is disabled.
    //
    // The system calls this member once when the OpMode is disabled.
    //--------
    // @Override
    public void exit() {
        elevatorThread.exit();
        intakeThread.exit();
        // imu= null;
    }



    private class ElevatorThread extends Thread {

      public ElevatorThread() {
        this.setName("ElevatorThread");
      }

      private volatile boolean exit = false;

      public void exit() {
        exit = true;
      }

      @Override
      public void run() {
        while(!exit) {
          elevator_update();
        }
      }

      private void elevator_update () {

        switch(wantedState) {
          case HOME : HomeElevator();
                      wantedState = WantedStates.STAY;
            break;

          case STACK : StackGlyphs();
                      wantedState = WantedStates.STAY;
            break;

          case SINGLE :  SingleStackGlyph();
                      wantedState = WantedStates.STAY;
            break;
          case RESTACK :  ReStackGlyphs();
                      wantedState = WantedStates.STAY;
            break;
          case RAISE_LOWER :  elevatorLowerRaise();
                      wantedState = WantedStates.STAY;
            break;
          case TILT_ARM_BACK :  tiltArmBack();
                      wantedState = WantedStates.STAY;
            break;
          case TILT_ARM_FORWARD :  tiltArmForward();
                      wantedState = WantedStates.STAY;
            break;
          case RELIC_NEXT_STEP :  relicNextStep();
                      wantedState = WantedStates.STAY;
            break;
          case RELIC_HOME :  relicHome();
                      wantedState = WantedStates.STAY;
            break;
        }

        if (elevatorState == ElevatorStates.LOW && !switchElevatorHome.getState()) { // Low and sitting on the home switch
          if (get_elevetor_position() > 5) { // but the encoder is showing higher 
            // reset encoder position to zero
            reset_elevator_encoder();
            elevator_run_to_position();
          };
        }


      }
    }


    public SystemState getSystemState() {
      return mSystemState;

    }

    private class IntakeThread extends Thread {
      public IntakeThread() {
        this.setName("IntakeThread");
      }

      private volatile boolean exit = false;

      @Override public void run() {
        while(!exit) {
            intake_update();
        }
      }

      public void exit() {
        exit = true;
      }
    }


    // -------------------------------------------------------------------------
    //
    // Get buttons from gamepad1
    public boolean getStackButton(){
      return gamepad1.y;
    }
    public boolean getHomeButton(){
      return gamepad1.a;
    }
    //--------------------------------------------------------------------------
    //
    // scale_motor_power
    //
    //--------
    // Scale the joystick input using a nonlinear algorithm.
    //--------
    double scale_motor_power (double p_power)
    {
        //
        // Assume no scaling.
        //
        double l_scale = 0.0f;

        //
        // Ensure the values are legal.
        //
        double l_power = Range.clip (p_power, -1, 1);

        double[] l_array =
            { 0.00, 0.05, 0.09, 0.10, 0.12
            , 0.16, 0.20, 0.24, 0.28, 0.30
            , 0.36, 0.43, 0.50, 0.72, 0.85
            , 1.00, 1.00
            };
        // double[] l_array =
        //     { 0.00, 0.05, 0.09, 0.10, 0.12
        //     , 0.15, 0.18, 0.24, 0.30, 0.36
        //     , 0.43, 0.50, 0.60, 0.72, 0.85
        //     , 1.00, 1.00
        //     };       //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int) (l_power * 16.0);
        if (l_index < 0)
        {
            l_index = -l_index;
        }
        else if (l_index > 16)
        {
            l_index = 16;
        }

        if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
        else
        {
            l_scale = l_array[l_index];
        }

        return l_scale;

    } // PushBotManual::scale_motor_power

    //--------------------------------------------------------------------------
    //
    // a_left_drive_power
    //
    //--------
    // Access the left drive motor's power level.
    //--------
    double a_left_drive_power ()
    {
        return motorLeft.getPower ();

    } // PushBotManual::a_left_drive_power

    //--------------------------------------------------------------------------
    //
    // a_right_drive_power
    //
    //--------
    // Access the right drive motor's power level.
    //--------
    double a_right_drive_power ()
    {
        return motorRight.getPower ();

    } // PushBotManual::a_right_drive_power

    //--------------------------------------------------------------------------
    //
    // set_drive_power
    //
    //--------
    // Scale the joystick input using a nonlinear algorithm.
    //--------
    public void set_drive_power (double p_left_power, double p_right_power)
    {
        motorLeft.setPower (p_left_power);
        motorRight.setPower (p_right_power);

    } // PushBotManual::set_drive_power

    //--------------------------------------------------------------------------
    //
    // run_using_encoders
    //
    /**
     * Sets both drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_using_encoders ()

    {
        DcMotor.RunMode l_mode
            = motorLeft.getMode();
        if (l_mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        {
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        l_mode = motorRight.getMode();
        if (l_mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        {
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    } // PushBotAuto::run_using_encoders

    //--------------------------------------------------------------------------
    //
    // reset_drive_encoders
    //
    /**
     * Resets both drive wheel encoders.
     */
    public void reset_drive_encoders ()

    {
        //
        // Reset the motor encoders on the drive wheels.
        //
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    } // PushBotAuto::reset_drive_encoders

    //--------------------------------------------------------------------------
    //
    // a_left_encoder_count
    //
    //--------
    // Access the left encoder's count.
    //--------
    int a_left_encoder_count ()
    {
        return motorLeft.getCurrentPosition ();

    } // PushBotManual::a_left_encoder_count

    //--------------------------------------------------------------------------
    //
    // a_right_encoder_count
    //
    //--------
    // Access the right encoder's count.
    //--------
    int a_right_encoder_count ()
    {
        return motorRight.getCurrentPosition ();

    } // PushBotManual::a_right_encoder_count

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reached
    //
    //--------
    // Scale the joystick input using a nonlinear algorithm.
    //--------
    boolean have_drive_encoders_reached
        ( double p_left_count
        , double p_right_count
        )
    {
        //
        // Assume failure.
        //
        boolean l_status = false;

        //
        // Have the encoders reached the specified values?
        //
        // TODO Implement stall code using these variables.
        //
        if ((Math.abs (motorLeft.getCurrentPosition ()) > p_left_count) &&
            (Math.abs (motorRight.getCurrentPosition ()) > p_right_count))
        {
            //
            // Set the status to a positive indication.
            //
            l_status = true;
        }

        //
        // Return the status.
        //
        return l_status;

    } // PushBotManual::have_encoders_reached

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reset
    //
    //--------
    // Scale the joystick input using a nonlinear algorithm.
    //--------
    boolean have_drive_encoders_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_status = false;

        //
        // Have the encoders reached zero?
        //
        if ((a_left_encoder_count () == 0) && (a_right_encoder_count () == 0))
        {
            //
            // Set the status to a positive indication.
            //
            l_status = true;
        }

        //
        // Return the status.
        //
        return l_status;

    } // PushBotManual::have_drive_encoders_reset


    public void jewelArmDown (){
      jewelServo.setPosition(JEWEL_DOWN);

    }

    public void jewelArmDown2 (){
      jewelServo.setPosition(JEWEL_DOWN2);

    }

    public void jewelArmUp (){
      jewelServo.setPosition(JEWEL_UP);

    }
    public int GetBlue(){
      return colorSensor.blue();

    }
    public int GetRed(){
      return colorSensor.red();
    }
    public void set_intake_power(double p_left_power, double p_right_power){
      intakeLeft.setPower(p_left_power);
      intakeRight.setPower(p_right_power);
    }


    public void intakeReverse() {
      intake_WantedState = Intake_WantedStates.REVERSE;
    }

    public void intakeToggleReverse() {
      if (intakeState == IntakeStates.STOP) {
          intake_WantedState = Intake_WantedStates.REVERSE;
      } else if (intakeState == IntakeStates.FORWARD  || intakeState == IntakeStates.REVERSE) {
          intake_WantedState = Intake_WantedStates.STOP;
      }
    }

    public void intakeToggle() {
      if (intakeState == IntakeStates.STOP) {
          if (mSystemState == SystemState.GLYPHS_HIGH || mSystemState == SystemState.GLYPHS_LOW ) {
            // return arm to HOME position; ready to received glyph
            wantedState = WantedStates.HOME;
          } else if (elevatorState == ElevatorStates.LOW) {
            // home position; open arm to receive glyph
            clawHome();
          }
          intake_WantedState = Intake_WantedStates.FORWARD;
      } else if (intakeState == IntakeStates.FORWARD) {
          intake_WantedState = Intake_WantedStates.STOP;
      }
    }

    public void intake_on(){
      intake_WantedState = Intake_WantedStates.FORWARD;
    }

    public void intake_off(){
      intake_WantedState = Intake_WantedStates.STOP;
    }

    public boolean get_switch_left_front(){
      return !switchLeftFront.getState();
    }

    public boolean get_switch_right_front(){
      return !switchRightFront.getState();
    }

    public boolean get_alignment_left(){
      return !alignmentLeft.getState();
    }
    public boolean get_alignment_right(){
      return !alignmentRight.getState();
    }
    public boolean get_glyph_switch(){
      return glyphSwitch.getState();
    }

    public void intake_update(){
      switch (intake_WantedState) {
        case STOP  : intakeState = IntakeStates.STOP;
            break;
        case FORWARD : intakeState = IntakeStates.FORWARD;
          break;
        case REVERSE : intakeState = IntakeStates.REVERSE;
          break;
      }


      if(intakeState == IntakeStates.FORWARD){
        if (get_switch_left_front()){
          intakeSpinGlyph();
        }
        else if (get_switch_right_front()){
          intakeSpinGlyph();
        }
        else if (get_alignment_left()){
          left_intake_power = LEFT_ALIGNMENT_INTAKE_POWER;

        }
        else if (get_alignment_right()){
          right_intake_power = RIGHT_ALIGNMENT_INTAKE_POWER;
        }
        else {
          left_intake_power = INTAKE_POWER;
          right_intake_power = INTAKE_POWER;
        }
        // if the glyph_switch is triggered while the intake is on, start the stacking process
        if(get_glyph_switch()){
          wantedState = WantedStates.STACK;
        }

      } else if (intakeState == IntakeStates.REVERSE){
        left_intake_power = -0.8;
        right_intake_power = -0.8;

      } else  {
        left_intake_power = 0;
        right_intake_power = 0;
      }

      set_intake_power(left_intake_power, right_intake_power);
    }

    private void intakeSpinGlyph() {
        left_intake_power = FRONT_SWITCH_INTAKE_POWER;
        right_intake_power = -FRONT_SWITCH_INTAKE_POWER;
        set_intake_power(-.4, -.5);
        sleep(50);
        set_intake_power(left_intake_power, right_intake_power);
        sleep(100);
    }

    public boolean getIntakeOn() {
      return (intakeState == IntakeStates.FORWARD);
    }

    //Elevator sequence
    int get_arm_position()
    {
        return armRotate.getCurrentPosition ();

    }


    private void tiltArmBack() {
      if (mSystemState == SystemState.GLYPHS_LOW || mSystemState == SystemState.GLYPHS_HIGH) {
        set_arm_state(ArmStates.TILT_BACK);
      }
    }

    //Elevator sequence
    private void tiltArmForward() {
      if (mSystemState == SystemState.GLYPHS_LOW || mSystemState == SystemState.GLYPHS_HIGH) {
        set_arm_state(ArmStates.TILT_FORWARD);
      }
    }

    public boolean switchElevatorHome() {
      return !switchElevatorHome.getState();
    }

    public void HomeElevator(){
      if (mSystemState == SystemState.HOME) { return;} ;

      clawHome();
      set_elevator_position(ElevatorStates.HIGH);
      set_arm_state(ArmStates.FRONT);
      set_elevator_position(ElevatorStates.LOW);
      mSystemState = SystemState.HOME ;
    }

    public void RunIntake(double leftPower, double rightPower){
      intakeLeft.setPower(leftPower);
      intakeRight.setPower(rightPower);
    }

    public void RealignSequence(){
      motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      RunIntake(-0.8, -0.8);
      DriveForwardEncoder(0.3, InchesToTicks(-8));
      // RunIntake(0.8);
      // DriveForwardEncoder(0.2,InchesToTicks(16));
      RunIntake(0, 0);
      //end
    }

    public void ReStackGlyphs(){
      set_elevator_position(ElevatorStates.HIGH);
      set_arm_state(ArmStates.FRONT);
      set_elevator_position(ElevatorStates.RESTACK);
      clawOpen();
      set_elevator_position(ElevatorStates.LOW);
      clawClose();
      set_elevator_position(ElevatorStates.HIGH);
      set_arm_state(ArmStates.BACK);
      mSystemState = SystemState.GLYPHS_HIGH;
    }

    public void SingleStackGlyph(){
      set_elevator_position(ElevatorStates.HIGH);
      set_arm_state(ArmStates.BACK);
      mSystemState = SystemState.GLYPHS_HIGH;
    }

    public void StackGlyphs(){
     if(mSystemState == SystemState.HOME) {
        clawClose();
        sleep(100);
        intakeReverse();
        set_elevator_position(ElevatorStates.STACK);
        intake_off();
        mSystemState = SystemState.GLYPHS_1;
      } else if(mSystemState == SystemState.GLYPHS_1) {
          // holding a glyph, load the next gylph
        set_elevator_position(ElevatorStates.HALF);
        clawHome();
        set_elevator_position(ElevatorStates.LOW);
        clawClose();
        sleep(100);
        intakeReverse();
        set_elevator_position(ElevatorStates.HIGH);
        set_arm_state(ArmStates.BACK);
        intake_off();
        mSystemState = SystemState.GLYPHS_HIGH;
      } else {
        wantedState = WantedStates.HOME;
      }
    }

    //Elevator and Arm Functions
    public void elevatorLowerRaise() {
      if (mSystemState == SystemState.GLYPHS_HIGH){
          set_elevator_position(ElevatorStates.LOW);
          mSystemState = SystemState.GLYPHS_LOW;
      } else if (mSystemState == SystemState.GLYPHS_LOW){
          set_elevator_position(ElevatorStates.HIGH);
          mSystemState = SystemState.GLYPHS_HIGH;
      } else if (mSystemState == SystemState.GLYPHS_1){
          set_elevator_position(ElevatorStates.LOW);
          mSystemState = SystemState.HOME;
          sleep(500);
          clawOpen();
      } else if (mSystemState == SystemState.HIGH){
          set_elevator_position(ElevatorStates.LOW);
          mSystemState = SystemState.HOME;
      } else if (mSystemState == SystemState.LOW  || mSystemState == SystemState.HOME){
          set_elevator_position(ElevatorStates.HIGH);
          mSystemState = SystemState.HIGH;
      }
    }

    private void reset_elevator_encoder(){
      elevatorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void elevator_run_to_position(){
      elevatorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void set_elevator_position(ElevatorStates p_ElevatorState){
      if (elevatorState == p_ElevatorState) { return; };

      elevatorState = p_ElevatorState;
      int m_Position = elevatorPositionTicks[p_ElevatorState.ordinal()];
      elevatorLift.setTargetPosition(m_Position);
      if (p_ElevatorState ==ElevatorStates.LOW) { // going to low state? look for the home switch

        while (elevatorLift.isBusy()  && switchElevatorHome.getState()) {
        }
        if (!switchElevatorHome.getState()) { // did we ust hit the homeswitch
          sleep(200);
          reset_elevator_encoder();
          elevator_run_to_position();
        }

      } else { // wait for the elevator to stop moving
        while (elevatorLift.isBusy()) {
        }
      }
    }

    int get_elevetor_position()
    {
        return elevatorLift.getCurrentPosition ();

    }


    public void relicHome() {
      relicClawClose();
      relicWristHome();
      set_relic_arm_position(relicArmPositionTicks[RelicArmStates.HOME.ordinal()]);
      relicArmState = RelicArmStates.HOME;
    }

    public void relicExtendArm() {
      relicClawOpen();
      relicWristExtended();
      set_relic_arm_position(relicArmPositionTicks[RelicArmStates.EXTENDED.ordinal()]);
      relicArmState = RelicArmStates.EXTENDED;
    }

    public void relicCapture() {
      relicClawClose();
      //wait for servo to close
      sleep(1000);
      set_relic_arm_position(relicArmPositionTicks[RelicArmStates.HOME.ordinal()]);
      //relicWristHome();
      relicArmState = RelicArmStates.CAPTURED;
    }

    public void relicPlaced() {
      set_relic_arm_position(relicArmPositionTicks[RelicArmStates.EXTENDED.ordinal()]);
      relicWristPlace();
      relicArmState = RelicArmStates.PLACED;
    }

    public void relicRetract() {
      relicClawOpen();
      sleep(200);
      set_relic_arm_position(relicArmPositionTicks[RelicArmStates.EXTENDED.ordinal()]-30);
      sleep(200);
      set_relic_arm_position(relicArmPositionTicks[RelicArmStates.EXTENDED.ordinal()]-400);
      relicWrist.setPosition(.7);
      set_relic_arm_position(relicArmPositionTicks[RelicArmStates.HOME.ordinal()]);
      relicArmState = RelicArmStates.RETRACTED;
    }

    public void relicNextStep() {
      switch (relicArmState) {
        case HOME  : relicExtendArm();
          break;
        case EXTENDED  : relicCapture();
          break;
        case CAPTURED  : relicWristTipped();
          break;
        case TIPPED  : relicPlaced();
          break;
        case PLACED  : relicRetract();
          break;
        case RETRACTED  : relicHome();
          break;
        //default : relicHome();

      }

    }


    public void set_relic_arm_position(int m_Position){
      relicArm.setTargetPosition(m_Position);
      while (relicArm.isBusy()) {
      }
    }

    public void relicClawClose() {
      relicClaw.setPosition(0);
    }

    public void relicClawOpen() {
      relicClaw.setPosition(0.38);
    }

    public void relicWristHome() {
      relicWrist.setPosition(0);
    }

    public void relicWristExtended() {
      relicWrist.setPosition(.07);
    }

    public void relicWristPlace() {
      relicWrist.setPosition(0.04);
    }

    public void relicWristTipped() {
      relicWrist.setPosition(.7);
      relicArmState = RelicArmStates.TIPPED;
    }

    //Arm Functions
    private void reset_arm_encoder(){
      armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void arm_run_to_position(){
      armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void set_arm_position(int p_position) {
      armRotate.setTargetPosition(p_position);
      while (armRotate.isBusy()) {
      }
    }

    private void set_arm_state(ArmStates p_ArmState) {
      if (armState == p_ArmState) { return; };
      armState = p_ArmState;
      int m_position = armPositionTicks[p_ArmState.ordinal()];
      armRotate.setTargetPosition(m_position);
      while (armRotate.isBusy()) {
      }
    }

    private void set_claw_state(ClawStates p_ClawState) {
      clawState = p_ClawState;
      double m_right_arm_position = clawPositionTicks[p_ClawState.ordinal()];
      double m_left_arm_position = (1-m_right_arm_position);
      set_claw_position(m_left_arm_position,m_right_arm_position);
    }

    public void clawOpen() {
      if (mSystemState == SystemState.HOME) {
        clawState = ClawStates.OPEN_REGULAR;
      } else if (clawState == ClawStates.CLOSE ) {
          clawState = ClawStates.OPEN_SLIGHTLY;
      } else if (clawState == ClawStates.OPEN_SLIGHTLY ) {
          clawState = ClawStates.OPEN_REGULAR;
      } else if (clawState == ClawStates.OPEN_REGULAR ) {
          clawState = ClawStates.OPEN_WIDE;
      } else  { return; }

      set_claw_state(clawState);
    }

    public void clawHome() {
      set_claw_state(ClawStates.OPEN_REGULAR);
    }

    public void clawClose(){
      clawState = ClawStates.CLOSE;
      set_claw_state(clawState);
    }

    private void set_claw_position(double p_left_arm_position, double p_right_arm_position){
      armLeft.setPosition(p_left_arm_position);
      armRight.setPosition(p_right_arm_position);
    }
    //Drive Functions
    public void DriveForwardEncoder(double power, int distance){
      motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motorLeft.setTargetPosition(distance);
      motorRight.setTargetPosition(distance);

      DriveForward(power);
      while(motorLeft.isBusy() && motorRight.isBusy()){
        //wait for the motors to reach the target
      }

      StopDriving();
      motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void EnableDriveEncoder(){
      motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void DisableDriveEncoder(){
      motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetDriveTargetPosition(int position){
      motorLeft.setTargetPosition(position);
      motorRight.setTargetPosition(position);
    }

    public void EnableDriveEncoder_Right() {
      motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    }
    public void DisableDriveEncoder_Right(){
      motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void SetDriveTargetPosition_Right(int position){
      motorRight.setTargetPosition(position);
    }

    public boolean isMotorLeftBusy(){
      return motorLeft.isBusy();
    }

    public boolean isMotorRightBusy(){
      return motorRight.isBusy();
    }

    public void DriveForward(double power){
      motorLeft.setPower(power);
      motorRight.setPower(power);
    }
    
    public void StopDriving(){
      motorLeft.setPower(0);
      motorRight.setPower(0);
    }


    public int InchesToTicks(int inches){
      return (int) (inches * INCH_2_TICKS_FACTOR);
    }

    public void sleep(int msec) {
      try {
          Thread.sleep(msec);
        } catch (InterruptedException e ) {};
    }
     //start of REV Robotics IMU functions and formatting
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading-", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle)
                    +":last-"+ formatAngle(angles.angleUnit, lastAngles.firstAngle)
                    +":global-"+ globalAngle;
                    }
                });

    }
     String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    //End of REV robotics functions and formattings
}
