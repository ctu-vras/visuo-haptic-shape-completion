% solve_chol - solve linear equations from the Cholesky factorization.
% Solve A*X = B for X, where A is square, symmetric, positive definite. The
% input to the function is R the Cholesky decomposition of A and the matrix B.
% Example: X = solve_chol(chol(A),B);
%
% NOTE: The program code is written in the C language for efficiency and is
% contained in the file solve_chol.c, and should be compiled using matlabs mex
% facility. However, this file also contains a (less efficient) matlab
% implementation, supplied only as a help to people unfamiliar with mex. If
% the C code has been properly compiled and is avaiable, it automatically
% takes precendence over the matlab code in this file.

function x = SolveCholesky(A, B)

if nargin ~= 2 || nargout > 1
  error('Wrong number of arguments.');
end

if size(A,1) ~= size(A,2) || size(A,1) ~= size(B,1)
  error('Wrong sizes of matrix arguments.');
end
opts.TRANSA = true; 
opts.UT     = true;
temp        = linsolve(A,B,opts);
opts.TRANSA = false;
x           = linsolve(A,temp,opts);
%x = A\(A'\B);
