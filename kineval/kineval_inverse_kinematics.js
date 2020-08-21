
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {
    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();
    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);
    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
          Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
          + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
          + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );
    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
//    console.log(endeffector_target_world);
    var h = robot.joints[endeffector_joint].xform;
//    console.log(endeffector_position_local);
//    console.log(endeffector_target_world);
//    console.log(h[0]);
//    console.log(h[1]);
//    console.log(h[2]);
//    console.log(h[3]);
    // transform local position into world frame
    var endeffector_position_world = matrix_multiply(h, endeffector_position_local);
    var euler = [0, 0, 0];
//    var xg = 0.9;
//    var yg = -0.6;
//    var zg =  1.6;
//    var xm = generate_rotation_matrix_X(xg);
//    var ym = generate_rotation_matrix_Y(yg);
//    var zm = generate_rotation_matrix_Z(zg);
//    var tm = matrix_multiply(matrix_multiply(zm, ym), xm);
//    if(Math.abs(tm[2][0]) != 1){
//        euler[1] = -Math.asin(tm[2][0]); // theta
////        euler[0] = Math.atan2(h[2][2] / Math.cos(euler[1]), h[2][1] / Math.cos(euler[1]));// psi
////        euler[2] = Math.atan2(h[0][0] / Math.cos(euler[1]), h[1][0] / Math.cos(euler[1]));// phi
//        euler[0] = Math.atan2(tm[2][1] / Math.cos(euler[1]), tm[2][2] / Math.cos(euler[1]));// psi
//        euler[2] = Math.atan2(tm[1][0] / Math.cos(euler[1]), tm[0][0] / Math.cos(euler[1]));// phi
//    }
//    else{
//        euler[2] = 0;
//        if(tm[2][0] == -1){
//            euler[1] = Math.PI / 2;
////            euler[0] = euler[2] + Math.atan2(h[0][2], h[0][1]);
//            euler[0] = euler[2] + Math.atan2(tm[0][1], tm[0][2]);
//        }
//        else{
//            euler[1] = -Math.PI / 2;
//            //euler[0] = -euler[2] + Math.atan2(-h[0][2], -h[0][1]);
//            euler[0] = -euler[2] + Math.atan2(-tm[0][1], -tm[0][2]);
//        }
//    }
//    console.log("euler[0] = ", euler[0]);
//    console.log("euler[1] = ", euler[1]);
//    console.log("euler[2] = ", euler[2]);
    // euler[0] = psi
    // euler[1] = theta
    // euler[2] = phi
//    euler[1] = Math.asin(h[1][0]);
//    if(h[1][0] == 1){
//        euler[0] = atan2(h[0][2], h[2][2]);
//        euler[2] = 0;
//    }
//    else if(h[1][0] == -1){
//        euler[0] = atan2(h[0][2], h[2][2]);
//        euler[2] = 0;
//    }
//    else{
//        euler[0] = Math.atan2(h[2][0], h[0][0]);
//        euler[2] = Math.atan2(-h[1][2], h[1][1]);
//    }
    //
//    euler[0] = Math.atan2(h[2][1], h[2][2]);
//    euler[1] = Math.asin(-h[2][0]);
//    euler[2] = Math.atan2(h[1][0], h[0][0]);
    //
    if(Math.abs(h[2][0]) != 1){
        euler[1] = -Math.asin(h[2][0]); // theta
//        euler[0] = Math.atan2(h[2][2] / Math.cos(euler[1]), h[2][1] / Math.cos(euler[1]));// psi
//        euler[2] = Math.atan2(h[0][0] / Math.cos(euler[1]), h[1][0] / Math.cos(euler[1]));// phi
        euler[0] = Math.atan2(h[2][1] / Math.cos(euler[1]), h[2][2] / Math.cos(euler[1]));// psi
        euler[2] = Math.atan2(h[1][0] / Math.cos(euler[1]), h[0][0] / Math.cos(euler[1]));// phi
    }
    else{
        euler[2] = 0;
        if(h[2][0] == -1){
            euler[1] = Math.PI / 2;
//            euler[0] = euler[2] + Math.atan2(h[0][2], h[0][1]);
            euler[0] = euler[2] + Math.atan2(h[0][1], h[0][2]);
        }
        else{
            euler[1] = -Math.PI / 2;
            //euler[0] = -euler[2] + Math.atan2(-h[0][2], -h[0][1]);
            euler[0] = -euler[2] + Math.atan2(-h[0][1], -h[0][2]);
        }
    }
    //console.log(euler);

    // compute e(x) for 6 dims
    var err = [];
    for(var i = 0; i < 3; i++){
        err[i] = endeffector_target_world.position[i] - endeffector_position_world[i][0];
    }
    for(var i = 3; i < 6; i++){
        if(kineval.params.ik_orientation_included)
            err[i] = endeffector_target_world.orientation[i - 3] - euler[i - 3];
        else
            err[i] = 0;
    }
    
    // compute Jacobian T
    var curr = endeffector_joint;
    var idx = 0;
    var jacobian = [];
    while(true){
        // extract rotational matrix
        var rot = []
        for(var i = 0; i < 3; i++){
            rot[i] = [];
            for(var j = 0; j < 3; j++){
                rot[i][j] = robot.joints[curr].xform[i][j];
            }
        }

        var z = matrix_multiply(rot, robot.joints[curr].axis);
        z = vector_normalize(z);

        //console.log(z);

        if(robot.joints[curr].type == "prismatic")
            jacobian[idx] = [z[0], z[1], z[2], 0, 0, 0]
        else{
            var offset = [endeffector_position_world[0][0] - robot.joints[curr].xform[0][3],
                          endeffector_position_world[1][0] - robot.joints[curr].xform[1][3],
                          endeffector_position_world[2][0] - robot.joints[curr].xform[2][3]];
            var Jv = vector_cross(z, offset);
            jacobian[idx] = [Jv[0], Jv[1], Jv[2], z[0], z[1], z[2]];
        }
        idx++;
        if(robot.joints[curr].parent == robot.base)
            break;
        curr = robot.links[robot.joints[curr].parent].parent;
    }

    var ctrl = 0;
    if(kineval.params.ik_pseudoinverse){
        var jacobian_orig = matrix_transpose(jacobian);
        var jacobian_inv = matrix_pseudoinverse(jacobian_orig);
        ctrl = matrix_multiply(jacobian_inv, err);
    }
    else{
        ctrl = matrix_multiply(jacobian, err);
    }
    curr = endeffector_joint;

    // apply control
    idx = 0;
    while(true){
        robot.joints[curr].control = kineval.params.ik_steplength * ctrl[idx++];
        if(robot.joints[curr].parent == robot.base)
            break;
        curr = robot.links[robot.joints[curr].parent].parent;
    }
}



