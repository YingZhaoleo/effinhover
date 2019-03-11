function visualize(T, X, skip)
% VISUALIZE Animation of hovercraft trajectory 
%   VISUALIZE(T, X, skip) displays an animation of the hovercraft's trajectory
%   (T, X). For skip = 1 every element of the trajectory is shown (slow).
%   To increase animation speed increase skip.
    
    h_size = 0.15;      % hovercraft size
    % Define hovercraft shape
    hovercraft = polyshape([-h_size/2, h_size/2, h_size/2*1.5, h_size/2*1.5, h_size/2, -h_size/2],... 
                           [-h_size/2,-h_size/2, -h_size/3, h_size/3, h_size/2, h_size/2]);

    hovercraft_plot = [];

    figure('position', [700, 100, 600, 600]);
    plot(X(:,1), X(:,2))
    title('Hovercraft trajectory', 'FontSize', 16)
    set(gca, 'YDir','reverse')      % Flip direction of y-axis to match coordinate system
    pos_ax = get(gca, 'Position');
    axis equal
    hold on

    % Create handle for text annotation to display time
    time_disp = annotation('textbox', pos_ax,...
                           'string', sprintf('time : %6.2f s', T(1)), ...
                           'EdgeColor', 'none', 'FontSize', 16);

    for k = 1:skip:size(T)
        delete(hovercraft_plot)
        hovercraft_plot = plot(translate(rotate(hovercraft, rad2deg(X(k,3))), X(k, 1), X(k, 2)),...
                               'FaceColor', [0.8, 0, 0]);
        set(time_disp, 'string', sprintf('time : %6.2f s', T(k)))
        pause(0.005)
    end

end
