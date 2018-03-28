package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

//------------------------------------------------------------------------------
//
// PushBotManual
//

/**
 * Extends the PushBotTelemetry and PushBotHardware classes to provide a basic
 * manual operational mode for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
 @TeleOp

public class RobotManual extends RobotTelemetry
{
    //--------------------------------------------------------------------------
    //
    // PushBotManual
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    
        private boolean prev_right_trig_val = false;
        private boolean right_trig_val = false;
        private boolean left_bumper = false;
        private boolean prev_left_bumper = false;
        private boolean right_bumper = false;
        private boolean prev_right_bumper = false;
        private boolean dpad_down = false,prev_dpad_down = false;

    
    public RobotManual ()
    {
        //
        // Initialize base classes.
        //
        // All via self-construction.
       

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotManual::PushBotManual
    


    //--------------------------------------------------------------------------
    //
      // loop
    //
    //--------
    // Initializes the class.
    //
    // The system calls this member repeatedly while the OpMode is running.
    //--------
    @Override public void runOpMode() throws InterruptedException
    {
      //
      // Call the PushBotHardware (super/base class) start method.
      //
      super.runOpMode();
        //----------------------------------------------------------------------
        //
        // DC Motors
        //
        // Obtain the current values of the joystick controllers.
        //
        // Note that x and y equal -1 when the joystick is pushed all of the way
        // forward (i.e. away from the human holder's body).
        //
        // The clip method guarantees the value never exceeds the range +-1.
        //
        // The DC motors are scaled to make it easier to control them at slower
        // speeds.
        //
        // The setPower methods write the motor power values to the DcMotor
        // class, but the power levels aren't applied until this method ends.
        //

        //
        // Manage the drive wheel motors.
        //
        
        waitForStart();

        while (opModeIsActive()) {
            float l_gp1_left_stick_y = -gamepad1.left_stick_y;
            float l_drive_power
                = (float)scale_motor_power (l_gp1_left_stick_y);
    
            float l_gp1_right_stick_x = -gamepad1.right_stick_x;
            float l_turn_power
                = (float)scale_motor_power (l_gp1_right_stick_x);
    
           // if (wantedState != WantedStates.STACK) {
                set_drive_power (l_drive_power - l_turn_power , l_drive_power + l_turn_power);
            //}
    
    
    
    
    
            right_trig_val = (gamepad1.right_trigger > 0.2);
            left_bumper = (gamepad1.left_bumper);
            right_bumper = (gamepad1.right_bumper);
            dpad_down = gamepad1.dpad_down;
            
            //intake motor bumpers on and off
            if (left_bumper && !prev_left_bumper){
              intakeToggle();
            } else if (right_bumper && !prev_right_bumper){
               intakeToggleReverse();
            } else if(gamepad1.a){
                wantedState = WantedStates.HOME;
            } else if (gamepad1.y){
                wantedState = WantedStates.STACK;
            } else if (right_trig_val && !prev_right_trig_val){
                clawOpen();
            } else if (gamepad1.left_trigger > 0.2){
                clawClose();
            } else if (gamepad1.b){
                wantedState = WantedStates.RAISE_LOWER;
            } else if (gamepad1.x){
                wantedState = WantedStates.SINGLE;
            } else if (dpad_down && !prev_dpad_down){
                  wantedState = WantedStates.RELIC_NEXT_STEP;
            } else if (gamepad1.dpad_up){
                  wantedState = WantedStates.RELIC_HOME;
            } else if (gamepad1.dpad_left){
                  wantedState = WantedStates.TILT_ARM_FORWARD;
            } else if (gamepad1.dpad_right){
                  wantedState = WantedStates.TILT_ARM_BACK;
            }
            
            prev_right_trig_val = right_trig_val;
            prev_left_bumper = left_bumper;
            prev_right_bumper = right_bumper;
            prev_dpad_down = dpad_down;
    
    
            update_telemetry (); // Update common telemetry
    
            telemetry.addData ("05", "System State:" + mSystemState);
            telemetry.addData ("06", "Wanted State:" + wantedState);
            telemetry.addData ("07", "Relic Arm State: " + relicArmState);
            telemetry.update();
            
        }            

        super.exit();
    } // PushBotManual::loop
} // PushBotManual
