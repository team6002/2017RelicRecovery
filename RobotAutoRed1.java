package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import for vuforia
  import org.firstinspires.ftc.robotcore.external.ClassFactory;
  import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
  import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
  import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
  import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
  import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
  import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
  import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
  import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
  import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
  import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
  import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
  import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
// end of import for vuforia

/**
 * Extends the PushBotTelemetry and PushBotHardware classes to provide a basic
 * autonomous operational mode for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
 @Autonomous
public class RobotAutoRed1 extends RobotTelemetry

{
    
    PIDController pidRotate;
    // Orientation             lastAngles = new Orientation();
    // double                  globalAngle, power = .30,correction;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    int pictograph = 1;// which column we aim for.
    boolean isBlue = false;
//end of vuforia class.

    //--------------------------------------------------------------------------

    public enum AutoState {
        SECURE //close arm lift glyph
        , READ //read the pictograph
        , JEWEL_ARM_DOWN
        , COLOR //detect color
        , KNOCK_OFF
        , BASELINE //drive to baeline depending on column
        , TURN // turn towards correct column based on pictograph
        , APPROACH
        , LOWER
        , RELEASE //
        , HOME
        , BACK // slightly back off
        , POSITION
        , END
        , TEST
    }
    private AutoState autoState = AutoState.SECURE;
    /**
     * Constructs the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public RobotAutoRed1 ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotAuto::PushBotAuto

    //--------------------------------------------------------------------------
    //
    // start
    //
    /**
     * Performs any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void runOpMode() throws InterruptedException
    {
      super.runOpMode();

        // start vuforia initialize block start.
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AQvU/4j/////AAAAGZN4bP6NVUx8qWwLodWRm7pQxbeqxZNcdILcGDbWqtdZ1Z5wXHbQxmMTXL/EP6rFa3raB4YzOoxDBlIBrgitMHFn1pnOUFbwLZLc7b2u+9Md64V85HGUTgJWhdP7ZP6X6+/XuDUw36gA69tBcrPDvHUVUxnTj6n7JZhjttv94Nzswi6649jSCZGctvGcgqL/7I+jKHqzUI0YDlKqXPTpbtV/YPq151qr+WqhlzmBeDGzfYHGVQGLUR36NapRsKtEbm2obgHQH9lVXLpkcbfdZb+DCkiIiraNOsOJefRzuSwyrlu26W1HrVgXs6oZRkReFF5J0RK5WIMKmV5ZZAQqmCm2rky0aYa+xuhYSuPC3YMY";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        // end of vuforia initialization block

      resetAngle();
      pidRotate = new PIDController(.0075 , .0035, 0);
      
      reset_drive_encoders ();
 
        waitForStart();
        
    //0 = unknown.
    //1 = right
    //2 = center
    //3 = left;
        relicTrackables.activate();

      boolean isAutoCompleted = false;
      while(opModeIsActive() && !isAutoCompleted) {
        getPictograph();
        autoPart1();
        switch (pictograph) {
          case 1: autoRightColumn();
                  break;
          case 2: autoCenterColumn();
                  break;
          case 3: autoLeftColumn();
                  break;
        }
        isAutoCompleted = true;
      }

      super.exit();

    } // PushBotAuto::start
    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
   
    
    private void autoRightColumn() {
      boolean isDone = false;
      autoState = AutoState.BASELINE;
      while(!isDone) {
        switch (autoState) {
                case BASELINE : driveStraight(34,.5);
                          autoState = AutoState.TURN;
              break;
              
          case TURN : rotate(54,.7);
                      autoState = AutoState.APPROACH;
              break;
          case APPROACH :wantedState = wantedState.SINGLE;
                      while(mSystemState != SystemState.GLYPHS_HIGH){}//wait for elevator to reach target.
                      autoState = AutoState.LOWER;
              break;
          case LOWER : wantedState = wantedState.RAISE_LOWER;
                      while(mSystemState != SystemState.GLYPHS_LOW){}//wait for elevator to reach target.
                      driveStraight(-10,.5);
                      autoState = AutoState.RELEASE;
              break;
          case RELEASE :clawOpen();clawOpen();sleep(500);
                      autoState = AutoState.HOME;
              break;
          case HOME : driveStraight(3,.5);
                      wantedState = wantedState.HOME;
                      while(mSystemState != SystemState.HOME){}//wait for elevator to reach target.
                      autoState = AutoState.BACK;
              break;    
          case BACK : driveStraight(-6,.5);driveStraight(3,.5);SimpleGyroTurn(30,1.0);
                      autoState = AutoState.POSITION;
              break;
          case POSITION :driveStraight(15,.5);SimpleGyroTurn(-180,1.0);
                  autoState = AutoState.END;
              break;
          case END : isDone=true;
              break;
        }

        // update_telemetry (); // Update common telemetry
        // telemetry.addData ("05", "System State:" + mSystemState);
        // telemetry.addData ("06", "Relic Arm State: " + relicArmState);        
        // telemetry.addData ("07", "Auto State:" + autoState);   
      }
    }


    private void autoCenterColumn() {
      boolean isDone = false;
      autoState = AutoState.BASELINE;
      while(!isDone) {
        switch (autoState) {
                case BASELINE : driveStraight(29,.5);
                          autoState = AutoState.TURN;
              break;
          case TURN : rotate(54,.7);driveStraight(-4,.5);rotate(54,.7);
                      autoState = AutoState.APPROACH;
              break;
          case APPROACH : 
                      wantedState = wantedState.SINGLE;
                      while(mSystemState != SystemState.GLYPHS_HIGH){}
                      autoState = AutoState.LOWER;
              break;
          case LOWER : 
                      wantedState = wantedState.RAISE_LOWER;
                      while(mSystemState != SystemState.GLYPHS_LOW){}//wait for elevator to reach target.
                      driveStraight(-6,.5);
                      autoState = AutoState.RELEASE;
              break;
          case RELEASE :clawOpen();clawOpen();sleep(500);
                      autoState = AutoState.HOME;
              break;
          case HOME : driveStraight(3,.5);
                      wantedState = wantedState.HOME;
                      while(mSystemState != SystemState.HOME){}//wait for elevator to reach target.
                     autoState = AutoState.BACK;
              break;    
          case BACK : driveStraight(-6,.5);driveStraight(3,.5);SimpleGyroTurn(-30,1.0);
                      autoState = AutoState.POSITION;
              break;
          case POSITION : driveStraight(15,.5);SimpleGyroTurn(180,1.0);
                  autoState = AutoState.END;
              break;
          case END : isDone=true;
              break;
        }

        // update_telemetry (); // Update common telemetry
        // telemetry.addData ("05", "System State:" + mSystemState);
        // telemetry.addData ("06", "Relic Arm State: " + relicArmState);        
        // telemetry.addData ("07", "Auto State:" + autoState);   
      }
    }

    private void autoLeftColumn() {
      boolean isDone = false;
      autoState = AutoState.BASELINE;
      while(!isDone) {
        switch (autoState) {
                case BASELINE : driveStraight(38,.5);
                          autoState = AutoState.TURN;
              break;
          case TURN : rotate(60,.7);driveStraight(-4,.5);rotate(45,.7);
                      autoState = AutoState.APPROACH;
              break;
          case APPROACH : 
                      wantedState = wantedState.SINGLE;
                      while(mSystemState != SystemState.GLYPHS_HIGH){}//wait for elevator to reach target.
                      autoState = AutoState.LOWER;
              break;
          case LOWER : wantedState = wantedState.RAISE_LOWER;
                      while(mSystemState != SystemState.GLYPHS_LOW){}//wait for elevator to reach target.
                      driveStraight(-7,.5);
                      autoState = AutoState.RELEASE;
              break;
          case RELEASE : clawOpen();clawOpen();sleep(500);
                      autoState = AutoState.HOME;
              break;
          case HOME : driveStraight(3,.5);
                      wantedState = wantedState.HOME;
                      while(mSystemState != SystemState.HOME){}//wait for elevator to reach target.
                      autoState = AutoState.BACK;
              break;    
          case BACK : driveStraight(-5,.5);driveStraight(3,.5);SimpleGyroTurn(30,1.0);
                      autoState = AutoState.POSITION;
              break;
          case POSITION : driveStraight(15,.5);SimpleGyroTurn(-170,1.0);
                  autoState = AutoState.END;
              break;
          case END : isDone=true;
              break;
        }

        // update_telemetry (); // Update common telemetry
        // telemetry.addData ("05", "System State:" + mSystemState);
        // telemetry.addData ("06", "Relic Arm State: " + relicArmState);        
        // telemetry.addData ("07", "Auto State:" + autoState);   
      }
    }

    private void autoPart1() {
      boolean isDone = false;
      autoState = AutoState.SECURE;
      while (!isDone) { // 
        switch (autoState) {
          case SECURE : wantedState = wantedState.STACK;
                        autoState = AutoState.COLOR;
              break;
          case COLOR : detectBlue();
                        autoState = AutoState.KNOCK_OFF;
              break;
          case KNOCK_OFF : redJewelKnockOff(isBlue);
                        autoState = AutoState.BASELINE;
              break;
          case BASELINE : autoState = AutoState.END;
                          sleep(500);
          case END : isDone=true;
              break;
        }
        update_telemetry (); // Update common telemetry
        telemetry.addData ("05", "System State:" + mSystemState);
        telemetry.addData ("06", "Relic Arm State: " + relicArmState);        
        telemetry.addData ("07", "Auto State:" + autoState);   
      } 
    }

    private int getPictograph() {
      boolean isDone = false;//start a timer
      ElapsedTime eTime = new ElapsedTime();
      eTime.reset();
      while (isDone == false) {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
          if (vuMark == RelicRecoveryVuMark.RIGHT){
            pictograph = 1;
          }
          else if (vuMark == RelicRecoveryVuMark.CENTER){
            pictograph = 2;
          }
          else if (vuMark == RelicRecoveryVuMark.LEFT){
            pictograph = 3;
          }
          isDone = true;
        }
        if (eTime.time() > 5.0) isDone = true;

      }

      telemetry.addData("pictograph", pictograph);
      telemetry.update();
      
      return pictograph;      
    }

    public void redJewelKnockOff(boolean blue){
      //if the color sensor sees blue, drive into it. Else drive away.
      jewelArmDown();
      if(blue){
        driveStraight(-5,.25);
        sleep(1000);
        jewelArmUp();
        driveStraight(8,.3);
        driveStraight(6,.25);
        sleep(500);
      }
      else{
        driveStraight(5,.25);
        sleep(500);
        jewelArmUp();
      }
    }


    public boolean detectBlue(){
      jewelArmDown();
      sleep(500);
      if(GetBlue() > GetRed() && GetBlue() > 15){
        isBlue = true;
        return isBlue; //1 is blue ball in front of sensor
      }
      
      jewelArmDown2();
      sleep(500);
      if(GetBlue() > GetRed() && GetBlue() > 15){
        isBlue = true;
        return isBlue; //1 is blue ball in front of sensor
      }

      // no blue detected
      isBlue = false;
      return isBlue;
      
    }
    

    public void driveStraight(int distance, double power){
      DisableDriveEncoder();
      resetAngle();
      EnableDriveEncoder();
      SetDriveTargetPosition(InchesToTicks(distance));
      set_drive_power(.5,.5); 
      // sleep(1000);
      while(isMotorRightBusy()){
        //wait for robot to rach target.
      }
      StopDriving();
      DisableDriveEncoder();
    }
    



    public void SimpleGyroTurn(int degree, double power){
        double feedForward = 0.25;
        double kP = 0.002;
      DisableDriveEncoder();
      resetAngle();
      
      double m_Power = 0.0;
      telemetry.update();
      if (degree < 0){ // right turn
        while (getAngle() > degree){
          m_Power = feedForward + ((globalAngle - degree)*kP);
          m_Power = Range.clip(m_Power,feedForward,power);
          set_drive_power(m_Power, -m_Power);
        }
      }
      else { //left turn
        while (getAngle() < degree){
          m_Power = feedForward + ((degree - globalAngle)*kP);
          m_Power = Range.clip(m_Power,feedForward,power);
          set_drive_power(-m_Power, m_Power);
        }
      }
      StopDriving();
    }
    
    
    

    private void resetAngle()
    {
        //lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //AxesOrder.ZYX
        telemetry.update();
        lastAngles = angles;
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      
        telemetry.update();
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
     private void rotate(int degrees, double power)
     {
         // // restart imu angle tracking.
         resetAngle();
         DisableDriveEncoder();
      

         // start pid controller. PID controller will monitor the turn angle with respect to the
         // target angle and reduce power as we approach the target angle with a minimum of 20%.
         // This is to prevent the robots momentum from overshooting the turn after we turn off the
         // power. The PID controller reports onTarget() = true when the difference between turn
         // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
         // The minimum power is determined by testing and must enough to prevent motor stall and
         // complete the turn. Note: if the gap between the starting power and the stall (minimum)
         // power is small, overshoot may still occur. Overshoot is dependant on the motor and
         // gearing configuration, starting power, weight of the robot and the on target tolerance.

         pidRotate.reset();
         pidRotate.setSetpoint(degrees);
         pidRotate.setInputRange(0, 90);
         pidRotate.setOutputRange(.22, power);
         pidRotate.setTolerance(3);
         pidRotate.enable();

         // getAngle() returns + when rotating counter clockwise (left) and - when rotating
         // clockwise (right).

         // rotate until turn is completed.

         if (degrees < 0)
         {
             // On right turn we have to get off zero first.
             if (getAngle() == 0) //opModeIsActive() && getAngle() == 0
             {
                 set_drive_power(power, -power);
                 sleep(100);
             }

             do
             {
                 power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                 set_drive_power(-power, power);
             }while(!pidRotate.onTarget()); //opModeIsActive() && !pidRotate.onTarget()
         }
         else    // left turn.
              set_drive_power(-power, power);
              sleep(100);
             do
             {
                 power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                 set_drive_power(-power, power);
             }while(!pidRotate.onTarget()); //opModeIsActive() && !pidRotate.onTarget()

         // turn the motors off.
         StopDriving();

         // wait for rotation to stop.
         sleep(500);

         // reset angle tracking on new heading.
         //resetAngle();
     }




} // PushBotAuto





