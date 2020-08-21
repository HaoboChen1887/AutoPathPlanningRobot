//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

function quaternion_from_axisangel(joint){
    var theta = joint.angle;
    var axis = joint.axis;
    var q = [Math.cos(theta / 2), 0, 0, 0]
    for(var i = 1; i < q.length; i++){
        q[i] = axis[i - 1] * Math.sin(theta / 2);
    }
    return q
}

function quaternion_normalize(q){
    return vector_normalize(q);
}

function quaternion_to_rotation_matrix(q){
    var ru = 
        [[1-2*(q[2]**2 + q[3]**2),  2*(q[1]*q[2]-q[0]*q[3]),    2*(q[0]*q[2]+q[1]*q[3]),    0],
        [2*(q[1]*q[2]+q[0]*q[3]),   1-2*(q[1]**2 + q[3]**2),    2*(q[2]*q[3]-q[0]*q[1]),    0],
        [2*(q[1]*q[3]-q[0]*q[2]),   2*(q[0]*q[1]+q[2]*q[3]),    1-2*(q[1]**2 + q[2]**2),    0],
        [0,                         0,                          0,                          1]];
    return ru;
}

function quaternion_multiply(q1, q2){
    product = [0, 0, 0, 0];
    product[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    product[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    product[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    product[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    return product;
}
    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

