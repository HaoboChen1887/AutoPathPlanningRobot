
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }
    else{
        kineval.buildFKTransforms();
    }

    // STENCIL: implement kineval.buildFKTransforms();

}

kineval.buildFKTransforms = function buildFKTransforms() {
	traverseFKBase();
}

function traverseFKBase(){
	var rot_x = generate_rotation_matrix_X(robot.origin.rpy[0]);
	var rot_y = generate_rotation_matrix_Y(robot.origin.rpy[1]);
	var rot_z = generate_rotation_matrix_Z(robot.origin.rpy[2]);
	var rot = matrix_multiply(matrix_multiply(rot_z, rot_y), rot_x);
	var trans = generate_translation_matrix(robot.origin.xyz[0],
											robot.origin.xyz[1],
											robot.origin.xyz[2]);
	robot.links[robot.base].xform = matrix_multiply(trans, rot);

    // extract heading and lateral
    robot_heading = matrix_multiply(robot.links[robot.base].xform, [[0], [0], [1] ,[1]]);
    robot_lateral = matrix_multiply(robot.links[robot.base].xform, [[1], [0], [0] ,[1]]);

	if(robot.links_geom_imported){
        // rotate to match with ROS cooridnates
		var rot_xx = generate_rotation_matrix_X(-Math.PI / 2);
		var rot_yy = generate_rotation_matrix_Y(-Math.PI / 2);
		var rot_ros = matrix_multiply(rot_yy, rot_xx);
		robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform, rot_ros);
	}
	for(var i = 0; i < robot.links[robot.base].children.length; i++){
		traverseFKJoint(robot.links[robot.base].children[i]);
	}
}

function traverseFKLink(link){
    robot.links[link].xform = robot.joints[robot.links[link].parent].xform;
    if(typeof robot.links[link].children === 'undefined'){
        return
    }
    for(var i = 0; i < robot.links[link].children.length; i++){
        traverseFKJoint(robot.links[link].children[i]);
    }
}

function traverseFKJoint(joint){
    var curr = robot.joints[joint];
	var rot_x = generate_rotation_matrix_X(curr.origin.rpy[0]);
	var rot_y = generate_rotation_matrix_Y(curr.origin.rpy[1]);
	var rot_z = generate_rotation_matrix_Z(curr.origin.rpy[2]);
	var rot = matrix_multiply(matrix_multiply(rot_z, rot_y), rot_x);
	var trans = generate_translation_matrix(curr.origin.xyz[0],
											curr.origin.xyz[1],
											curr.origin.xyz[2]);
    
    var type = curr.type;
    var ru = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangel(curr)));

    switch(type){
        case "revolute":
        case "continuous":
            break;
        case "prismatic":
            // angle means position along the axis
            ru = generate_translation_matrix(curr.axis[0] * curr.angle,
                                             curr.axis[1] * curr.angle,
                                             curr.axis[2] * curr.angle);
            break;
        case "fixed":
            ru = generate_identity(4);
            break;
    }
    var xform_p = robot.links[curr.parent].xform;
    curr.xform = matrix_multiply(matrix_multiply(matrix_multiply(xform_p, trans), rot), ru);

    traverseFKLink(robot.joints[joint].child);
}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

