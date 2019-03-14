function [x_next] = EM(X,U,h,f)
% EM: Euler method integration
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns :
%    State h seconds in the future

k1 = f(X, U);

x_next = X + h*(k1);
end