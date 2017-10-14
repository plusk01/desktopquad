function rots

    % Create the original coordinate axes
    kk = linspace(0,1,100);
    CX = [kk; zeros(1,length(kk)); zeros(1,length(kk))];
    CY = [zeros(1,length(kk)); kk; zeros(1,length(kk))];
    CZ = [zeros(1,length(kk)); zeros(1,length(kk)); kk];
    
    % Plot the world coordinate system
    figure(1), clf;
%     plotAxes(CX, CY, CZ);
    a = 5;
    plotAxes(CX, CY, CZ, [-1 -1 -1]*a);
    axis([-1 1 -1 1 -1 1]*a);
      
    % Base tf
%     q = eul2quat(deg2rad(fliplr([180 0 0]))); % rpy
    rot = rotx(90)'*rotz(180);
%     rot = quat2rotm(q);
    CXr = rot*CX;
    CYr = rot*CY;
    CZr = rot*CZ;
    T = [0 0 -(a-1)];
    plotAxes(CXr, CYr, CZr, T)
    
%     % ArUco (from top)
%     q = quatmultiply(q, eul2quat(deg2rad(fliplr([0 0 -90])))); % rpy
%     %rot = rotx(180)*roty(0)*rotz(0);
%     rot = quat2rotm(q);
%     CXr = rot*CX;
%     CYr = rot*CY;
%     CZr = rot*CZ;
%     T = [0 0 -(a-2.2)];
%     plotAxes(CXr, CYr, CZr, T)
%     
%     % Camera (from top)
%     q = quatmultiply(q, eul2quat(deg2rad(fliplr([180 0 -90])))); % rpy
%     %rot = rotx(180)*roty(0)*rotz(0);
%     rot = quat2rotm(q);
%     CXr = rot*CX;
%     CYr = rot*CY;
%     CZr = rot*CZ;
%     T = [0 0 -(a-3.4)];
%     plotAxes(CXr, CYr, CZr, T)

end

function plotAxes(CX, CY, CZ, T, linestyle)

    % Should a specific linestyle be applied?
    if nargin < 5, ls = '-'; else ls = linestyle; end;
    
    % Does the user want this coordinate frame translated?
    if nargin < 4, T = []; end;
    if isempty(T), T = [0 0 0]; end;
    if size(T, 1) == 1, T = T.'; end;

    % Scale the largest coordinate axes value by this number.
%     scale = 2;
%     sf = max(CX(:))*scale;
%     if sf <= 0, sf = 1; end;
    
    % Translate the origin of the coordinate system
    CX = repmat(T, 1, size(CX,2)) + CX;
    CY = repmat(T, 1, size(CY,2)) + CY;
    CZ = repmat(T, 1, size(CZ,2)) + CZ;

    % Plot the 3D coordinate axes
    hold on;
    plot3(CX(1,:), CX(2,:), CX(3,:),'color','r','linewidth',2,'linestyle',ls);
    plot3(CY(1,:), CY(2,:), CY(3,:),'color','g','linewidth',2,'linestyle',ls);
    plot3(CZ(1,:), CZ(2,:), CZ(3,:),'color','b','linewidth',2,'linestyle',ls); 
    view(-30, 30); axis square; grid on;
%     axis([-1 1 -1 1 -1 1]*sf);
%     xlabel('X'); ylabel('Y'); zlabel('Z');
end