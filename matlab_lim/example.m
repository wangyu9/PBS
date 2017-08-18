% ---------------------------------------------------------
% 2d example with two triangles and two constraint vertices
% ---------------------------------------------------------
% v : #vertices
% d : dimension, 3 for triangles and 4 for tets
% c : #positional constraints
% ---------------------------------------------------------
% vx3 matrix of vertices of mesh
vertices = [0 0 0; 0 1 0; 1 1 0; 1 0 0];
% vx3 matrix of vertices of rest pose mesh
initialVertices = vertices;
% vxd matrix of indices of elements of mesh (3 for triangle meshes, 4 for tet meshes)
elementIdx = [2 1 3; 4 3 1];
% (only needed for 2D LSCM) column vector of border vertex indices
borderVertices = 0;
% (only needed for 2D Poisson) vector containing partial derivatives of target element gradients
% structure: [xx_1, xy_1, xx_2, xy_2, ..., xx_v, xy_v, yx_1, yy_1, yx_2, yy_2, ..., yx_v, yy_v]'
gradients = 0;
% matrix containing triplets of sparse linear consraint matrix C of positional constraints (Cv-d)
% dimension: (c)x(v*d)
constraintMatrix = [1 1 1; 2 2 1; 3 3 1; 4 4 1];
% column vector d of target positional constraints (Cv-d)
% dimension: v*d
constraintTarget = [10 5 0 1]';
% deformation energy: 0=Dirichlet, 1=Laplacian, 2=Green, 3=ARAP or (only 2D) 4=LSCM, 5=Poisson
energy = 2;
% max squared positional constraint error
tolerance = 1e-6;
% max number of iterations
maxIterations = 100;
% iterate until local minima is found
findLocalMinima = 1;
% enable solver output
enableOutput = 1;
% enable alpa updating
enableAlphaUpdate = 1;
% barrier weight (-1 default: ARAP/LSCM = 0.01, Green = 1)
beta = -1;
% min area/volume of each element (-1 default: 1e-5 * smallest triangle)
eps = -1;


% solve until tolerance is fulfilled and or local minima is found
[deformedVertices] = lim_solver_mex(vertices,initialVertices,elementIdx, ...
                        borderVertices,gradients,constraintMatrix,constraintTarget, ...
                        energy,tolerance,maxIterations,findLocalMinima, ...
                        enableOutput,enableAlphaUpdate,beta,eps)

% init solver
lim_solver_mex_step(vertices,initialVertices,elementIdx,borderVertices,gradients, ...
    constraintMatrix,constraintTarget,energy,enableOutput,enableAlphaUpdate,beta,eps);
for i=1:10
   % compute one iteration
  [deformedVertices] = lim_solver_mex_step(vertices)
end

% -----------------------------------------------------------
% 3d example with one tetrahedron and two constraint vertices
% -----------------------------------------------------------
vertices = [0 0 0; 0 1 0; 1 1 0; 0 0 1];
initialVertices = vertices;
elementIdx = [2 1 3 4];
constraintMatrix = [1 1 1; 2 2 1; 3 3 1; 4 4 1; 5 5 1; 6 6 1];
constraintTarget = [-1 -1 0 0 1 0]';

% solve until tolerance is fulfilled and or local minima is found
[deformedVertices] = lim_solver_mex(vertices,initialVertices,elementIdx, ...
    borderVertices,gradients,constraintMatrix,constraintTarget,energy,tolerance, ...
    maxIterations,findLocalMinima,enableOutput,enableAlphaUpdate,beta,eps)

% init solver
lim_solver_mex_step(vertices,initialVertices,elementIdx,borderVertices,gradients, ...
    constraintMatrix,constraintTarget,energy,enableOutput,enableAlphaUpdate,beta,eps);
for i=1:10
  % compute one iteration
  [deformedVertices] = lim_solver_mex_step(vertices)
end
