//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}

function matrix_multiply(m1, m2) {
    var mat = [];
    var dim_1 = m1.length;
    var dim_2 = m2[0].length;

    if(m1[0].length !== m2.length){
        console.error("matrix dimensions must match");
        return mat;
    }
    if(typeof dim_2 === 'undefined'){
        for(var i = 0; i < dim_1; i++){
            mat[i] = 0;
            for(var j = 0; j < m2.length; j++){
                mat[i] += m1[i][j] * m2[j];
            }
        }
    }
    else{
        for(var i = 0; i < dim_1; i++){
            mat[i] = [];
            for(var j = 0; j < dim_2;j++){
                mat[i][j] = 0;
                for(var curr = 0; curr < m2.length; curr++){
                    mat[i][j] += (m1[i][curr] * m2[curr][j]);
                }
            }
        }
    }
//    for(var i = 0; i < dim_1; i++){
//        mat[i] = [];
//        for(var j = 0; j < dim_2;j++){
//            mat[i][j] = 0;
//            for(var curr = 0; curr < m2.length; curr++){
//                mat[i][j] += (m1[i][curr] * m2[curr][j]);
//            }
//        }
//    }
    return mat;
}

function matrix_transpose(m1){
    var m1_t = [];
    for(var j = 0; j < m1[0].length; j++){
        m1_t[j] = [];
    }
    for(var i = 0; i < m1.length; i++){
        for(var j = 0; j < m1[0].length; j++){
            m1_t[j][i] = m1[i][j];
        }
    }
    return m1_t;
}

function matrix_pseudoinverse(m1){
    var dim_1 = m1.length;
    var dim_2 = m1[0].length;
    var m1_plus = [];
    var m1_t = matrix_transpose(m1)
    if(dim_1 > dim_2){
        var ata = matrix_multiply(m1_t, m1);
        // this may cause error left for future
        m1_plus = matrix_multiply(numeric.inv(ata), m1_t);
    }
    else if(dim_1 < dim_2){
        var aat = matrix_multiply(m1, m1_t);
        // this may cause error left for future
        m1_plus = matrix_multiply(m1_t, numeric.inv(aat));
    }
    return m1_plus;
}

function vector_normalize(v1){
    var e1 = [];
    var sum = 0;
    for(var i = 0; i < v1.length; i++){
        sum += Math.pow(v1[i], 2);
    }
    sum = Math.sqrt(sum);
    for(var i = 0; i < v1.length; i++){
        e1[i] = v1[i] / sum;
    }
    return e1;
}

function vector_cross(v1, v2){
    var vec = [];
    vec[0] = v1[1] * v2[2] - v1[2] * v2[1];
    vec[1] = v1[2] * v2[0] - v1[0] * v2[2];
    vec[2] = v1[0] * v2[1] - v1[1] * v2[0];
    return vec;
}

function generate_identity(dim){
    var mat = [];
    for(var i = 0; i < dim; i++){
        mat[i] = [];
        for(var j = 0; j < dim; j++){
            if(i == j)
                mat[i][j] = 1;
            else
                mat[i][j] = 0;
        }
    }
    return mat;
}

function generate_translation_matrix(dx, dy, dz){
    var mat = [ [1, 0, 0, dx],
                [0, 1, 0, dy],
                [0, 0, 1, dz],
                [0, 0, 0,  1]];
    return mat;
}

function generate_rotation_matrix_X(theta){
    var mat = [ [1,                 0,                  0,                      0],
                [0,                 Math.cos(theta),    -Math.sin(theta),       0],
                [0,                 Math.sin(theta),    Math.cos(theta),        0],
                [0,                 0,                  0,                      1]];
    return mat;
}

function generate_rotation_matrix_Y(theta){
    var mat = [ [Math.cos(theta),   0,                  Math.sin(theta),        0],
                [0,                 1,                  0,                      0],
                [-Math.sin(theta),  0,                  Math.cos(theta),        0],
                [0,                 0,                  0,                      1]];
    return mat;
}

function generate_rotation_matrix_Z(theta){
    var mat = [ [Math.cos(theta),   -Math.sin(theta),   0,                      0],
                [Math.sin(theta),   Math.cos(theta),    0,                      0],
                [0,                 0,                  1,                      0],
                [0,                 0,                  0,                      1]];
    return mat;
}

    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse // TODO:matrix inverse needed
    //   matrix_invert_affine // TODO:not implemented 
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix 
    //   generate_rotation_matrix_X 
    //   generate_rotation_matrix_Y 
    //   generate_rotation_matrix_Z 

