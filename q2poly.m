% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
    theta1 = q(1);
    theta2 = q(2);

    R1 = [cos(theta1), -sin(theta1);
      sin(theta1), cos(theta1)];
    R2 = [cos(theta2), -sin(theta2);
      sin(theta2), cos(theta2)];

    pivot1 = robot.pivot1;
    pivot2 = pivot1 + (R1 * robot.pivot2);

    poly1 = R1 * robot.link1 + pivot1;
    poly1 = polyshape(poly1(1,:), poly1(2,:));
    poly2 = (R1 * R2 * robot.link2) + pivot2;
    poly2 = polyshape(poly2(1,:), poly2(2,:));

end