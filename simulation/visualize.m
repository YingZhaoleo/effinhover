function visualize(t, X, skip)
% VISUALIZE Animation of hovercraft trajectory 
% 
% Inputs: 
%   t : Time (row) vector
%   X : Matrix containing states (rows) for each sampling time in t (columns)
%   skip : Only show every skip'th sampling time to speed up animation

% Define hovercraft shape
h_size = 0.15;      % hovercraft size
hovercraft = polyshape([-h_size/2, h_size/2, h_size/2*1.5, h_size/2*1.5, h_size/2, -h_size/2],... 
                       [-h_size/2,-h_size/2, -h_size/3, h_size/3, h_size/2, h_size/2]);

hovercraft_plot = [];

figure('position', [700, 100, 600, 600]);
plot(X(1,:), X(2,:))
title('Hovercraft trajectory', 'FontSize', 16)
set(gca, 'YDir','reverse')      % Flip direction of y-axis to match coordinate system
pos_ax = get(gca, 'Position');
axis equal
hold on

% Create handle for text annotation to display time
time_disp = annotation('textbox', pos_ax,...
                       'string', sprintf('time : %6.2f s', t(1)), ...
                       'EdgeColor', 'none', 'FontSize', 16);
                                
for k = 1:skip:length(t)
    delete(hovercraft_plot)
    hovercraft_plot = plot(translate(rotate(hovercraft, rad2deg(X(3,k))), X(1, k), X(2, k)),...
                           'FaceColor', [0.8, 0, 0]);
    set(time_disp, 'string', sprintf('time : %6.2f s', t(k)))    
    pause(0.005)
end

end
