function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
route = 0;
X = start_coords(1);
Y = start_coords(2);
route(1,:) = [X, Y];        % the start of the route


step = sqrt(gx .^2 + gy.^2);
% the distance between successive locations in the route should not be greater than 1.0 (normalize).
gx = gx ./step;
gy = gy ./step;

for i = 1:max_its

    newX = X + gx(round(Y), round(X));
    newY = Y + gy(round(Y), round(X));
    if round(newX) <=size(f, 2) && round(newX) >=1
        X = newX;
    end
    if round(newY) <=size(f, 1) && round(newY) >=1
        Y = newY;
    end
    distGoal = sqrt((X - end_coords(1)) ^2 + (Y - end_coords(2)) ^2);

   if distGoal < 1.0
       return;
   end
   route(i+1,:) = [X, Y];
% *******************************************************************
end
