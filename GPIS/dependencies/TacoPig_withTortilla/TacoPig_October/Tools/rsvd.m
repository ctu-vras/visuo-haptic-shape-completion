%
% Computes the SVD decomposition of a low-rank square matrix
%
% Follows the algorithm described in
% Finding structure with randomness: Stochastic algorithms
% for constructing approximate matrix decompositions
% N. Halko, P.G. Martinsson, J. Tropp | arXiv.org report 0909.4061.
%
% [U,S,V] = rsvd(A, k, alg)
% A is a big matrix
% k is the rank
% alg is the algorithm, 'std', 'single', 'power', 'srft'
%    
% A is approximately U*S*V'; 
%
% Fabio Tozeto Ramos 03/03/2012
function [U,S,V] = rsvd(A,k,alg)
[D1,D2] = size(A);

if nargin < 3
    alg = 'std';
end

if D1 == D2
    %q        = 2; %accuracy enhanced
    %Y        = (A * A')^q * A * omega;
    if strcmp(alg,'srft')
        D       = diag((2 * rand(1,D1) - 1) + ...
                  1i*(2 * rand(1,D1) - 1));
        t       = cumsum(ones(D1));
        F       = D1^(-0.5)*...
                  exp(-2*pi*1i*(t - 1).*(t' - 1)/D1);      
        E       = zeros(D1, k);
        t       = randi(D1, 1, k);
        ind     = sub2ind([D1 k], t, 1:k);
        E(ind)  = 1;
        omega   = real(D * F * E);
    else
        omega   = 10*randn(D1, k);
    end

    Y        = A * omega;
    [Q,R]    = gs(Y);
    if strcmp(alg, 'std') || strcmp(alg, 'srft') 
        B       = Q' * A;
    elseif strcmp(alg, 'single')
        B       = (Q' * Y)/(Q' * omega); 
    end
    [Ut,S,V] = svd(B);
    U        = Q * Ut; 
    S        = S(:,1:k);  %fix dimensionality
    if strcmp(alg, 'std') || strcmp(alg, 'srft')
        V       = V(:,1:k);  %fix dimensionality 
    else
        V       = U;
    end
else
    %For retangular matrices
    if strcmp(alg,'srft')
%        D       = diag((2 * rand(1,D1) - 1) + ...
%                  1i*(2 * rand(1,D1) - 1));
%        t       = cumsum(ones(D1));
%        F       = D1^(-0.5)*...
%                  exp(-2*pi*1i*(t - 1).*(t' - 1)/D1);      
%        E       = zeros(D1, k);
%        t       = randi(D1, 1, k);
%        ind     = sub2ind([D1 k], t, 1:k);
%        E(ind)  = 1;
%        omega   = real(D * F * E);
        error('Not yet implemented for retangular matrices.');
    else
        p        = 0;  %over smapling parameter
        omega    = 10*randn(D2, k + p);
        Y        = A * omega;
        [Q,R]    = gs(Y);
        if strcmp(alg, 'std') || strcmp(alg, 'srft') 
            B        = Q' * A;
            [Ut,S,V] = svd(B);
            U        = Q * Ut; 
            S        = S(:,1:k);  %fix dimensionality
            V        = V(:,1:k);  %fix dimensionality 
        elseif strcmp(alg, 'single')
            psi      = 10*randn(D1, k + p);
            Z        = A' * psi; 
            [W,R]    = gs(Z);
            B1       = (Q' * Y)/(W' * omega); 
            B2       = ((W' * Z)/(Q' * psi))'; 
            %B        = [Q' * Y,  W' * Z]/[W' * omega,  Q' * psi];
            B        = 0.5*B1 + 0.5*B2;
            [Ut,S,Vt] = svd(B);
            U        = Q * Ut;
            V        = W * Vt;
            S        = S(:,1:k);  %fix dimensionality
            V        = V(:,1:k);  %fix dimensionality
            
        end
        
    end    
end

% Gram-Schmidt orthonormalisation
function [Q,R] = gs(A)
[m,n] = size(A);
% compute QR using Gram-Schmidt
for j = 1:n
   v  = A(:,j);
   for i=1:j-1
        R(i,j) = Q(:,i)'*A(:,j);
        v      = v - R(i,j)*Q(:,i);
   end
   R(j,j) = norm(v);
   Q(:,j) = v/R(j,j);
end