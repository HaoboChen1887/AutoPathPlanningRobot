//   CREATE ROBOT STRUCTURE

// KE 

links_geom_imported = false;

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "mr2";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,2,0], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "base";  

        
// specify and create data objects for the links of the robot
robot.links = {
    "base": {},  
    "upperarm_right": {}, 
    "upperarm_left": {}, 
    "leg_right": {},
    "leg_left": {},
	"head": {}
};
/* for you to do
, "shoulder_left": {}  , "upperarm_left": {} , "forearm_left": {} };
*/

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.right_shoulder = {parent:"base", child:"upperarm_right"};
robot.joints.right_shoulder.origin = {xyz: [0,1,-0.5], rpy:[-Math.PI/4,0,0]};
robot.joints.right_shoulder.axis = [0.0,1.0,0.0]; 

robot.joints.left_shoulder = {parent:"base", child:"upperarm_left"};
robot.joints.left_shoulder.origin = {xyz: [0,1,0.5], rpy:[Math.PI/4,0,0]};
robot.joints.left_shoulder.axis = [0.0,1.0,0.0]; 

robot.joints.left_hip = {parent:"base", child:"leg_right"};
robot.joints.left_hip.origin = {xyz: [0,-0.5,0.3], rpy:[3.5 * Math.PI/4,0,0]};
robot.joints.left_hip.axis = [0.0,1.0,0.0]; 

robot.joints.right_hip = {parent:"base", child:"leg_left"};
robot.joints.right_hip.origin = {xyz: [0,-0.5,-0.3], rpy:[Math.PI - 3.5 * Math.PI/4,0,0]};
robot.joints.right_hip.axis = [0.0,1.0,0.0]; 

robot.joints.neck = {parent:"base", child:"head"};
robot.joints.neck.origin = {xyz: [0,1,0], rpy:[0,0,0]};
robot.joints.neck.axis = [0.0,1.0,0.0]; 

// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "right_shoulder";
robot.endeffector.position = [[0],[0],[0.5],[1]]

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["base"] = new THREE.CubeGeometry( 0.5, 1.7, 1 );
links_geom["base"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.2, 0) );

links_geom["head"] = new THREE.CubeGeometry( 0.5, 0.5, 0.5 );
links_geom["head"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.25, 0) );

links_geom["upperarm_right"] = new THREE.CubeGeometry( 1.2, 0.2, 0.2 );
links_geom["upperarm_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0.65, 0, 0) );

links_geom["upperarm_left"] = new THREE.CubeGeometry( 1.2, 0.2, 0.2 );
links_geom["upperarm_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0.65, 0, 0) );

links_geom["leg_right"] = new THREE.CubeGeometry( 0.3, 1.3, 0.3 );
links_geom["leg_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.75, 0) );

links_geom["leg_left"] = new THREE.CubeGeometry( 0.3, 1.3, 0.3 );
links_geom["leg_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.75, 0) );
