
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    // debugger;
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);
    eps = 0.5;
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 2;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations

        if(rrt_alg == 1){
            q_rand = random_config();
            var status = rrt_extend(T_a, q_rand);
            var q_new = T_a.vertices[T_a.newest].vertex;
            if(status != "Trapped"){
                if(rrt_connect(T_b, q_new) == "Reached"){
                    // drawHighlightedPath(path_dfs(T_a));
                    // drawHighlightedPath(path_dfs(T_b));
                    var path_final = find_path(T_a, T_b);
                    draw_path(path_final);
                    kineval.motion_plan = path_final;
                    return "reached";
                }
            }
            var temp = T_a;
            T_a = T_b;
            T_b = temp;
            return "failed";
        }
        else if(rrt_alg == 2){
            q_rand = random_config();
            var status = rrt_star_extend(T_a, q_rand);
            console.log("offset " + calcDist(T_a.vertices[T_a.newest].vertex, q_goal_config));
            if(calcDist(T_a.vertices[T_a.newest].vertex, q_goal_config) < eps * 2){
                var path_final = path_dfs(T_a);
                draw_path(path_final);
                kineval.motion_plan = path_final;
                return "reached";
            }
            else
                return "failed";
        }

    }

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].cost = 0;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs
function draw_path(path_final){
    for(var i = 0; i < path_final.length; i++){
        path_final[i].geom.material.color = {r:0, g:0 ,b:1};
    }
}

function find_path(T_a, T_b){
    var path_a = path_dfs(T_a);
    var path_b = path_dfs(T_b);
    var path_final = [];
    var start_from_a = true;
    for(var i = 0; i < path_a[0].vertex.length; i++){
        if(path_a[0].vertex[i] != 0){
            start_from_a = false;
        }
    }
    if(!start_from_a){
        path_final = path_a.concat(path_b.reverse());
    }
    else{
        path_final = path_b.concat(path_a.reverse());
    }
    return path_final;
}

function rrt_extend(T, q){
    var prev = nearest_neighbor(q, T);
    var q_near = T.vertices[prev].vertex;
    var q_new = new_config(q, q_near, []);
    if(!kineval.poseIsCollision(q_new)){
        tree_add_vertex(T, q_new);
        tree_add_edge(T, prev, T.newest);
        T.vertices[T.newest].parent = T.vertices[prev];
        if(calcDist(q_new, q) <= eps / 2)
            return "Reached";
        else
            return "Advanced";
    }
    return "Trapped";
}

function rrt_star_extend(T, q){
    var prev = nearest_neighbor(q, T);
    var q_near = T.vertices[prev].vertex;
    var q_new = new_config(q, q_near, []);
    if(!kineval.poseIsCollision(q_new)){
        tree_add_vertex(T_a, q_new);
        var neighbors = find_neighbors(q_new, eps * 2);
        var best = choose_parent(neighbors, q_new)
        T_a.vertices[T_a.newest].parent = T_a.vertices[best[0]];
        T_a.vertices[T_a.newest].cost = best[1];
        rewire(neighbors, q_new, best[0]);
        if(calcDist(q_new, q) <= eps / 2)
            return "Reached";
        else
            return "Advanced";
    }
    return "Trapped";
}

function choose_parent(neighbors, q_new){
    if(neighbors.length == 0)
        return 0;
    var best = neighbors[0];
    var min_cost = T_a.vertices[neighbors[0]].cost + calcDist(T_a.vertices[neighbors[0]].vertex, q_new);
    for(var i = 1; i < neighbors.length; i++){
        var potential_cost = T_a.vertices[neighbors[i]].cost + calcDist(T_a.vertices[neighbors[i]].vertex, q_new);
        if(potential_cost < min_cost){
            min_cost = potential_cost;
            best = neighbors[i]
        }
    }
    return [best, min_cost];
}

function rewire(neighbors, q_new, best){
    for(var i = 0; i < neighbors.length; i++){
        if(neighbors[i] != best){
            var potential_cost = T_a.vertices[T_a.newest].cost + calcDist(T_a.vertices[neighbors[i]].vertex, q_new);
            if(potential_cost < T_a.vertices[neighbors[i]].cost){
                T_a.vertices[T_a.newest].cost = potential_cost;
                T_a.vertices[neighbors[i]].parent = T_a.vertices[T_a.newest];
            }
        }
    }
}

function rrt_connect(T, q){
    var status = rrt_extend(T, q);
    while(status == "Advanced"){
        status = rrt_extend(T, q)
    }
    return status;
}

function path_dfs(T){
    var path = [T.vertices[T.newest]];
    var curr = path[0];
    while(curr.parent != null){
        path.unshift(curr.parent);
        curr = curr.parent;
    }
    return path;
}

function random_config(){
    var q_goal = [];
    q_goal[0] = Math.random() * (robot_boundary[1][0] - robot_boundary[0][0]) + robot_boundary[0][0];
    q_goal[1] = robot.origin.xyz[1]; // fix y
    q_goal[2] = Math.random() * (robot_boundary[1][2] - robot_boundary[0][2]) + robot_boundary[0][2];
    q_goal[3] = 0;
    q_goal[4] = Math.random() * 2 * Math.PI - Math.PI // +-PI
    q_goal[5] = 0;

    for(var x in robot.joints){
        var idx = q_names[x];
        if(robot.joints[x].type == "fixed"){
            q_goal[idx] = 0;
        }
        else if(robot.joints[x].limit == undefined){
            q_goal[idx] = Math.random() * 2 * Math.PI - Math.PI;
        }
        else{
            q_goal[idx] = Math.random() * (robot.joints[x].limit.upper - robot.joints[x].limit.lower) + robot.joints[x].limit.lower;
        }
    }

    return q_goal;
}

function new_config(q, q_near, q_new){
    var dist = calcDist(q_near, q);
    var step = eps / dist;
    for(var i = 0; i < q.length; i++){
        q_new.push(q_near[i] + (q[i] - q_near[i]) * step);
    }
    return q_new;
}

function nearest_neighbor(q, T){
    var min_dist = calcDist(q, T.vertices[0].vertex);
    var min_idx = 0;
    for(var i = 1; i <= T.newest; i++){
        var curr_dist = calcDist(q, T.vertices[i].vertex);
        if(min_dist > curr_dist){
            min_dist = curr_dist;
            min_idx = i;
        }
    }
    return min_idx;
}

function calcDist(q1, q2){
    var dist = 0;
    for(var i = 0; i < q1.length; i++){
        if(i == 0 || i == 2)
            dist += Math.pow(q1[i] - q2[i], 2);
        if(i == 4)
            dist += Math.pow(0.2 * (q1[i] - q2[i]), 2);
    }
    return Math.sqrt(dist);
}

function find_neighbors(q, rad){
    var neighbors = []
    for(var i = 0; i < T_a.newest; i++){
        var curr = T_a.vertices[i].vertex;
        if(calcDist(q, curr) < rad){
            neighbors.push(i);
        }
    }
    return neighbors
}










