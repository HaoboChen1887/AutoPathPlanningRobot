
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    // STENCIL: implement FSM to cycle through dance pose setpoints
    // loop through the setpoints
    var curtime = Date.now();
    kineval.dt = (curtime - kineval.t_) / 1000.0;
    kineval.t_ = curtime;
    kineval.accu_time += kineval.dt;
    
    // only update setpoint when enough time is elapsed 
    if(kineval.accu_time > 0.6){
        kineval.params.dance_pose_index = kineval.params.dance_sequence_index.shift();
        kineval.params.dance_sequence_index.push(kineval.params.dance_pose_index);
        kineval.accu_time = 0;
    }

    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = kineval.setpoints[kineval.params.dance_pose_index][x];
    }
    kineval.t_ = curtime;
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    for(x in robot.joints) {
        robot.joints[x].control = robot.joints[x].servo.p_gain * (kineval.params.setpoint_target[x] - robot.joints[x].angle);
    }
    // STENCIL: implement P servo controller over joints
}


