package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//------------------------------------------------------------------------------
//
// PushBotTelemetry
//

/**
 * Extends the PushBotHardware class to provide basic telemetry for the Push
 * Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-02-13-57
 */
public class RobotTelemetry extends RobotHardware

{
    //--------------------------------------------------------------------------
    //
    // update_telemetry
    //
    /**
     * Update the telemetry with current values from the base class.
     */
    

    public void update_telemetry ()

    {
        //
        // Send telemetry data to the driver station.
        //

        // telemetry.addData
        //     ( "01"
        //     , "Left Drive: "
        //         + a_left_drive_power ()
        //         + ", "
        //         + a_left_encoder_count ()
        //     );
        // telemetry.addData
        //     ( "02"
        //     , "Right Drive: "
        //         + a_right_drive_power ()
        //         + ", "
        //         + a_right_encoder_count ()
        //     );
        // telemetry.addData
        //     ( "03"
        //     , "Elevator: "
        //         + "Home " + switchElevatorHome() 
        //         + "," + get_elevetor_position()
        //     );
        // telemetry.addData
        //     ( "04"
        //     , "Main Arm: "
        //         + get_arm_position()
        //     );
            
        
    } // PushBotTelemetry::loop



} // PushBotTelemetry
